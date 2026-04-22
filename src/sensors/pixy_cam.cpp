#include "pixy_cam.h"

void PixyCam::init() {
    pixy.init();
}

void PixyCam::update() {
    pixy.ccc.getBlocks();
}

BlockInfo PixyCam::emptyBlock() {
    BlockInfo b;
    b.detected = false;
    b.signature = 0;
    b.x = 0;
    b.y = 0;
    b.width = 0;
    b.height = 0;
    b.distanceEst = 999.0f;
    return b;
}

float PixyCam::estimateDistance(int blockHeight) {
    if (blockHeight <= 0) return 999.0f;
    // Pinhole model: dist = (REF_DIST_CM * REF_HEIGHT_PX) / measuredHeight
    return (REF_DIST_CM * REF_HEIGHT_PX) / (float)blockHeight;
}

BlockInfo PixyCam::findLargestBlock(int sig) {
    BlockInfo best = emptyBlock();
    int bestArea = 0;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == sig) {
            int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
            if (area > bestArea) {
                bestArea = area;
                best.detected = true;
                best.signature = sig;
                best.x = pixy.ccc.blocks[i].m_x;
                best.y = pixy.ccc.blocks[i].m_y;
                best.width = pixy.ccc.blocks[i].m_width;
                best.height = pixy.ccc.blocks[i].m_height;
                best.distanceEst = estimateDistance(best.height);
            }
        }
    }
    return best;
}

BlockInfo PixyCam::findLargestBlockTwoSigs(int sig1, int sig2) {
    BlockInfo best = emptyBlock();
    int bestArea = 0;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        int s = pixy.ccc.blocks[i].m_signature;
        if (s == sig1 || s == sig2) {
            int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
            if (area > bestArea) {
                bestArea = area;
                best.detected = true;
                best.signature = s;
                best.x = pixy.ccc.blocks[i].m_x;
                best.y = pixy.ccc.blocks[i].m_y;
                best.width = pixy.ccc.blocks[i].m_width;
                best.height = pixy.ccc.blocks[i].m_height;
                best.distanceEst = estimateDistance(best.height);
            }
        }
    }
    return best;
}

BlockInfo PixyCam::getClosestPillar() {
    // Return the largest red or green block (closest = largest in frame)
    return findLargestBlockTwoSigs(PIXY_SIG_RED, PIXY_SIG_GREEN);
}

BlockInfo PixyCam::getParkingMarker() {
    return findLargestBlock(PIXY_SIG_MAGENTA);
}