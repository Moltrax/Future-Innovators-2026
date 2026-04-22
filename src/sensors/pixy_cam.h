#pragma once
#include <Arduino.h>
#include <Pixy2SPI_SS.h>
#include "config.h"

struct BlockInfo {
    bool detected;
    int signature;      // 1=red, 2=green, 3=magenta
    int x;              // 0-315 horizontal
    int y;
    int width;
    int height;
    float distanceEst;  // cm, pinhole model
    
};

struct PixyCam {
    Pixy2SPI_SS pixy;

    void init();
    void update();
    BlockInfo getClosestPillar();
    BlockInfo getParkingMarker();

private:
    BlockInfo findLargestBlock(int sig);
    BlockInfo findLargestBlockTwoSigs(int sig1, int sig2);
    float estimateDistance(int blockHeight);
    BlockInfo emptyBlock();
};