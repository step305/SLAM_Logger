//
// Created by step305 on 21.07.2021.
//
#include "serialStream.h"

enum ParserStates {
    WAIT_DLE1,
    WAIT_DATA,
    WAIT_DLE2
};

class UARTParser{
public:
    UARTParser()
};
