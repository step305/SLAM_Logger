//
// Created by driver on 09.09.2021.
//

#include "vector_slam_gyro_data.h"

void project_point(float x, float y, const CamStruct &cam, float& u, float& v) {
    float r2 = x * x + y * y;
    float r4 = powf( r2, 2.0 );
    float r6 = powf( r2, 3.0 );
    float k1 = cam.kc[0];
    float k2 = cam.kc[1];
    float p1 = cam.kc[2];
    float p2 = cam.kc[3];
    float k3 = cam.kc[4];

    float xd = x * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) +
               2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
    float yd = y * (1.0f + k1 * r2 + k2 * r4 + k3 * r6) +
               p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;
    u =  cam.fc[0] * xd + cam.cc[0];
    v =  cam.fc[1] * yd + cam.cc[1];
}

//Hamming distance between two descriptors
// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int hammingDistance32(const int32_t *pa, const int32_t *pb)
{
    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

void SLAM( MapType &featureMap, Eigen::Matrix<float, Eigen::Dynamic,Eigen::Dynamic> &P,
        float q[4],
        Eigen::Matrix<float,3,1> &bw,
        Eigen::Matrix<float,3,1> &sw,
        Eigen::Matrix<float,6,1> &mw,
        SyncPacket &frame,
        const float dt,
        const CamStruct &cam,
        std::vector<cv::Point2f>  &erased ) {

    const int nxv = 15;
    const int nxf = 3;
    Eigen::Matrix<float,3,3> Cbn;
    Eigen::Matrix<float,3,3> Cnb;

    if (frame.sync == false) {
        Eigen::Matrix<float,3,1> dThe;
        dThe << 0.0f, 0.0f, 0.0f;
        // Attitude mechanization
        attitude_mechanization(q, frame.dangle, dt, Cbn, dThe, bw, sw, mw);
        Cnb = Cbn.transpose();

        // Kalman predict stage
        // System dynamics matrix F and system noise covariance matrix Q
        // Continuous-time system matrix
        Eigen::Matrix<float, nxv, nxv> A = Eigen::MatrixXf::Zero( nxv, nxv );
        A.block<3,3>(0,3) = -Cbn;
        A.block<3,3>(0,6) = -Cbn*dThe.asDiagonal();

        Eigen::Matrix<float,3,6> G = Eigen::MatrixXf::Zero( 3, 6 );
        G << dThe(1), dThe(2), 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, dThe(0), dThe(2), 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, dThe(0), dThe(1);
        A.block<3,6>(0,9) = -Cbn*G;

        // State transition matrix
        Eigen::Matrix<float,nxv,nxv> F = Eigen::MatrixXf::Identity( nxv, nxv );
        F += A * dt;
        Eigen::Matrix<float,nxv,nxv> FT;
        FT = F.transpose();

        // Attitude errors noise
        float na = 1e-4f;
        // Bias noise
        float nr = 1e-4f;
        // Scale noise
        float ns = 1e-7f;
        // Misalignment noise
        float nm = 1e-3f;
        Eigen::Matrix<float,nxv,1> diag;
        diag << na, na, na,
                nr, nr, nr,
                ns, ns, ns,
                nm, nm, nm, nm, nm, nm;
        // System noise
        Eigen::Matrix<float,nxv,nxv> Qn = Eigen::MatrixXf::Zero( nxv, nxv );
        Qn.diagonal() = diag;
        Eigen::Matrix<float,nxv,nxv> QnT;
        QnT = Qn.transpose();

        // Trapezioidal integration
        Eigen::Matrix<float,nxv,nxv> Q = Eigen::MatrixXf::Zero( nxv, nxv );
        Q = dt / 2.0f * ( F * Qn + QnT * FT );

        // Covariance predict:
        // Vehicle
        Eigen::Matrix<float,nxv,nxv> Ptemp;
        Ptemp = P.topLeftCorner( nxv, nxv );
        P.topLeftCorner( nxv, nxv ) = F * Ptemp * FT + Q;

        // Features
        if( featureMap.size() > 0) {
            //Eigen::Matrix<float,nxv, nxf * featureMap.size()> PtempMap;

            auto PtempMap = P.block( 0, nxv, nxv, nxf * featureMap.size());
            P.block( 0, nxv, nxv, nxf*featureMap.size() ) = F * PtempMap;

            PtempMap = P.block( 0, nxv, nxv, nxf*featureMap.size() );
            P.block( nxv, 0, nxf*featureMap.size(), nxv ) = PtempMap.transpose();
        }
    } else {
        // Measurements (Update State)
        quat2dcm(Cbn, q);
        Cnb = Cbn.transpose();

        //Containers for feature augmentation
        std::vector<Eigen::Matrix<float,3,1>> en_augment;
        std::vector<std::vector<int32_t>> ds_augment;
        en_augment.reserve(50);
        ds_augment.reserve(50);

        int nmatch = 0;

        std::vector<DescriptorType> descriptors = frame.descriptors;
        std::vector<PointCoordsType> points = frame.points;

        int npoints = points.size();

        int nstate = nxv + featureMap.size() * nxf;
        Eigen::Matrix<float, Eigen::Dynamic, 1> X( nstate, 1 );
        X = Eigen::MatrixXf::Zero( nstate, 1 );

        Eigen::Matrix<float,3,1> V;
        Eigen::Matrix<float,nxf,nxf> Cbneb = Eigen::MatrixXf::Zero( nxf, nxf );
        Eigen::Matrix<float,nxf,nxf> R = Eigen::MatrixXf::Identity( nxf, nxf );
        R *= meas_noise;

        Eigen::Matrix<float,Eigen::Dynamic,3> K( nstate, nxf );
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> KH( nstate, nstate );
        Eigen::Matrix<float,nxf,nxf> S;
        Eigen::Matrix<float,nxf,nxf> Sinv;
        Eigen::Matrix<float,3,1> en_map;
        Eigen::Matrix<float,3,1> eb_map;

        for (int meas = 0; meas < npoints; meas++) {
            // Keypoint unit vectors in b- and n-frames
            Eigen::Matrix<float,3,1> eb_mes = {
                    sqrtf( 1.0f - ( powf( points[meas].x, 2.0f ) +
                        powf( points[meas].y, 2.0f ) ) ),
                    points[meas].x,
                    points[meas].y
            };
            Eigen::Matrix<float,3,1> en_mes = Cbn * eb_mes;

            // Keypoint descriptor
            int32_t des_mes[8];
            memcpy(des_mes, descriptors[meas].val, sizeof(uint8_t) * 32);
            //get_descriptor32(descriptors, meas, des_mes);

            //Map management flag
            bool too_close  = false;
            bool feature_in_frame = false;

            float uf, vf;
            project_point(eb_mes(1), eb_mes(2), cam, uf, vf);

            if ( ( uf > excluded_band ) && (uf < (cam.nCols - excluded_band) ) &&
            (vf > excluded_band) && (vf < (cam.nRows - excluded_band) ) &&
            (eb_mes(0) > 0.9f ) ) {
                feature_in_frame = true;
            } else {
                feature_in_frame = false;
            }

            //Look for the matching feature on the map
            for( auto &feature : featureMap ) {
                // Check if the measured unit vector is not too close to the
                // one on the map
                // b-frame unit vector for map feature
                en_map = feature.en;
                //Map feature unit vector in b-frame
                //Project map feature to the camera matrix
                //Memoization
                float u, v;
                if ( !feature.vis ) {
                    //Map feature unit vector in b-frame
                    eb_map = Cnb * en_map;
                    feature.eb = eb_map;

                    //Project map feature to the camera matrix
                    project_point(eb_map(1), eb_map(2), cam, u, v);
                    feature.u = u;
                    feature.v = v;
                    feature.vis = true;
                } else {
                    eb_map = feature.eb;
                    u = feature.u;
                    v = feature.v;
                }

                //If map feature fits to camera and doesn't fall to the excluded band
                if ( feature.obs || ( ( u > excluded_band ) && (u < (cam.nCols - excluded_band) ) &&
                    (v > excluded_band) && (v < (cam.nRows - excluded_band) ) &&
                    (eb_map(0) > 0.9f ) )
                    ) {

                    //Feature observed
                    feature.obs = true;

                    //Check if the measured feature is close to the one on the map
                    //Feature and measurement angles
                    float theta_f = atan2f( en_map( 1 ), en_map( 0 ) );
                    float theta_m = atan2f( en_mes( 1 ), en_mes( 0 ) );
                    float phi_f = atan2f( en_map( 2 ),
                                          sqrtf( en_map(0) * en_map(0) + en_map(1) * en_map(1) ) );
                    float phi_m = atan2f( en_mes( 2 ),
                                          sqrtf( en_mes(0) * en_mes(0) + en_mes(1) * en_mes(1) ) );
                    if ( ( fabsf(theta_f - theta_m) < vector_closeness_threshold ) &&
                        ( fabsf(phi_f - phi_m) < vector_closeness_threshold ) ) {
                        too_close = true;
                    }

                    if ( nmatch < max_match ) {
                        if ( hammingDistance32( des_mes, feature.des ) < hamming_norm_threshold ) {
                            // Measurement vector and matrix
                            V = eb_mes - eb_map;

                            Cbneb(0,1) = -en_mes(2);
                            Cbneb(0,2) =  en_mes(1);
                            Cbneb(1,0) =  en_mes(2);
                            Cbneb(1,2) = -en_mes(0);
                            Cbneb(2,0) = -en_mes(1);
                            Cbneb(2,1) =  en_mes(0);
                            Eigen::Matrix<float,3,3>  Hv;
                            Eigen::Matrix<float,3,3>  Hf;
                            Hv = -Cnb * Cbneb;
                            Hf =  Cnb;
                            Eigen::Matrix<float,3,3>  HvT = Hv.transpose();
                            Eigen::Matrix<float,3,3>  HfT = Hf.transpose();

                            //Innovation Covariance
                            Eigen::Matrix<float,3,3> Pvv = P.topLeftCorner( 3, 3 );
                            Eigen::Matrix<float,3,3> Pvf = P.block<3,3>(0,feature.pos);
                            Eigen::Matrix<float,3,3> Pfv = P.block<3,3>(feature.pos,0);
                            Eigen::Matrix<float,3,3> Pff = P.block<3,3>(feature.pos,feature.pos);
                            S = ( Hv * Pvv + Hf * Pfv ) * HvT + ( Hv * Pvf + Hf * Pff ) * HfT + R;
                            Sinv = S.inverse();

                            //Check Chi2 threshold
                            float chi2 = V.transpose() * Sinv * V;

                            //If threshold was met
                            if ( chi2 < 6.25 ) {
                                //Matched   feature
                                feature.mat = true;
                                nmatch++;

                                //Kalman Gain
                                int pos = 0;
                                K = Eigen::MatrixXf::Zero( nstate, nxf );
                                KH = Eigen::MatrixXf::Zero( nstate, nstate );

                                while( pos < nstate ) {
                                    K.block( pos, 0, 3, 3 ) = (P.block( pos, 0, 3, 3 ) * HvT +
                                        P.block( pos, feature.pos, 3, 3 ) * HfT ) * Sinv;
                                    KH.block( pos, 0, 3, 3 ) = -K.block( pos, 0, 3, 3 ) * Hv;
                                    KH.block( pos, feature.pos, 3, 3 ) = -K.block( pos, 0, 3, 3 ) * Hf;
                                    pos += 3;
                                }
                                for (int k = 0; k < nstate; k++) {
                                    KH(k, k) += 1.0f;
                                }
                                auto KT = K.transpose();
                                auto KHT = KH.transpose();

                                //Update Covariance
                                P = KH * P * KHT + K * R * KT;
                                //Update State
                                X += K * V;

                                //Make P symmetric
                                auto PT = P.transpose();
                                P = 0.5 * ( P + PT );

                                break;
                            } // if chi2 ...
                        } // if hamming ...
                    } // if nmatch ...
                } // if feature.obs ...
            } // for feature in featureMap

            if ((!too_close) && feature_in_frame) {
                const int32_t* p = (int32_t*)&(descriptors[meas].val[0]);
                std::vector<int32_t> des(p, p + 8);
                en_augment.push_back(en_mes);
                ds_augment.push_back(des);
            }
        } // for meas ... (keypoints)

        // Correct Attitude Quaternion
        // Error quaternion
        float qe[4];
        qe[1] = X( 0 ) * 0.5f;
        qe[2] = X( 1 ) * 0.5f;
        qe[3] = X( 2 ) * 0.5f;
        float mgn = ( qe[1] * qe[1] + qe[2] * qe[2] + qe[3] * qe[3] );
        if( mgn < 1.0f ) {
            qe[0] = sqrtf( 1.0f - mgn );
            float qtemp[4];
            quat_mult( qtemp, q, qe );
            memcpy( q, qtemp, sizeof(float) * 4 );
        }

        // Update gyro bias estimate
        bw += X.block(3,0,3,1);

        // Update gyro scale estimate
        sw += X.block(6,0,3,1);

        // Update gyro misalignment estimate
        mw += X.block(9,0,6,1);

        // Correct N-frame unit vectors
        for( auto &feature : featureMap ) {
            feature.en += X.block( feature.pos, 0, nxf, 1 );
            float vec[3] = {feature.en(0), feature.en(1), feature.en(2)};
            feature.en = feature.en / vec_norm(vec, 3);
            feature.cnt_obs += feature.obs;
            feature.cnt_mat += feature.mat;
        }

        //MAP MANAGEMENT: Delete faulty features
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> P_( nstate, nstate );
        P_ = P;

        std::vector<int> index_keep{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 }; //always keep attitude and gyro state
        index_keep.reserve(nstate);

        int pos = 15;
        auto it = featureMap.begin( );

        while( it != featureMap.end( ) ) {
            float obs_rate = (float)((*it).cnt_mat) / (float)( (*it).cnt_obs );
            if ( obs_rate < obs_thr ) {
                erased.push_back( cv::Point2f( (*it).u, (*it).v ) );
                it = featureMap.erase(it);
            } else {
                index_keep.push_back((*it).pos);
                index_keep.push_back((*it).pos+1);
                index_keep.push_back((*it).pos+2);
                (*it).pos = pos;
                pos += 3;
                it++;
            }
        }
        nstate = nxv + featureMap.size() * nxf;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> P_new( nstate, nstate );
        P_new = P(index_keep, index_keep);
        P.resize( nstate, nstate );
        P = P_new;

        //MAP MANAGEMENT: Add new features
        if( nmatch < min_match) {
            unsigned int naugm = min_match - nmatch;
            naugm = ( naugm >= en_augment.size( ) ) ? en_augment.size( ) : naugm;
            for( unsigned int i = 0; ( ( i < naugm ) && ( featureMap.size() < max_map ) ) ; ++i ) {
                //New feature
                FeatureStruct feature;

                //Flags and counters
                feature.cnt_obs = 0;
                feature.cnt_mat = 0;
                feature.mat = false;
                feature.obs = false;
                feature.vis = false;

                //Feature unit vector
                Eigen::Matrix<float,3,1> en = en_augment[i];
                feature.en = en;

                //Feature ORB Descriptor
                std::vector<int32_t> des = ds_augment[i];
                for( unsigned int j=0; j < des.size(); ++j ) {
                    feature.des[j] = des[j];
                }

                //Covariance matrix rows and columns indices for the feature
                feature.pos = ( nxv + nxf * ( featureMap.size() + 1 ) ) - 3;

                //Add feature to the Map
                featureMap.push_back( feature );

                //Augment Covariance Matrix
                Eigen::Matrix<float,nxf,nxv> Hv = Eigen::MatrixXf::Zero( nxf, nxv );
                Eigen::Matrix<float,nxv,nxf> HvT;
                Hv(0,1) = -en(2);
                Hv(0,2) =  en(1);
                Hv(1,0) =  en(2);
                Hv(1,2) = -en(0);
                Hv(2,0) = -en(1);
                Hv(2,1) =  en(0);
                HvT = Hv.transpose();
                Eigen::Matrix<float,nxf,nxf> Hz = Cbn;
                Eigen::Matrix<float,nxf,nxf> HzT = Hz.transpose();

                //Add rows and columns to Covariance Matrix
                Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> P_new( P.rows() + nxf, P.cols() + nxf );
                P_new = Eigen::MatrixXf::Zero( P.rows() + nxf, P.cols() + nxf );
                P_new.topLeftCorner( P.rows(), P.cols() ) = P;
                P.resize( P_new.rows(), P_new.cols() );
                P = P_new;

                //Feature covariance
                P.block( feature.pos, feature.pos, nxf, nxf ) =
                    Hv * P.topLeftCorner( nxv, nxv ) * HvT + Hz * R * HzT;

                //Vehicle to feature cross-covariance
                P.block( feature.pos, 0, nxf, nxv ) = Hv * P.topLeftCorner(nxv, nxv );
                Eigen::Matrix<float, nxf, nxv> tempP = P.block( feature.pos, 0, nxf, nxv );
                P.block( 0, feature.pos, nxv, nxf ) = tempP.transpose();

                //Feature to feature cross-covariance
                auto start = featureMap.begin();
                auto stop = std::prev(featureMap.end() );
                Eigen::Matrix<float, nxf, nxf> tempPP;
                for( auto it = start; it != stop; ++it ) {
                    P.block( feature.pos, (*it).pos, nxf, nxf ) = Hv * P.block( 0, (*it).pos, nxv, nxf );
                    tempPP = P.block( feature.pos, (*it).pos, nxf, nxf );
                    P.block( (*it).pos, feature.pos, nxf, nxf ) = tempPP.transpose();
                }
            } //for ... featureMap
        } // match_cnt < min_match
    } // if ... else ... frame.sync
}