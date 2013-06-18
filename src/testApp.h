#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKinectInpainter.h"
#include "ofxBlobTracker.h"

#include "ofxUI.h"
#include "ofxXmlSettings.h"

#define BACKGROUND_FRAMES 100

#define MAP_POINTS 4

class testApp : public ofBaseApp{
public:

	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed (int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    void touchAdded(ofxBlob &_blob);
    void touchMoved(ofxBlob &_blob);
    void touchDeleted(ofxBlob &_blob);

    void getBackground();

	ofxKinect kinect;
    ofxBlobTracker          touchTracker;
    ofxKinectInpainter inPainter;
    ofFloatPixels background;
    ofxCvGrayscaleImage backgroundTex;
    // for KinectBlobTracker
    ofxCvGrayscaleImage thresMask;
    int dilate;
    int erode;

    float nearThreshold;
    float farThreshold;

    float touchDiffFarThreshold;
    float touchDiffNearThreshold;

	bool learnBackground;
	int backFrames;
	int angle;

	int minBlobPoints;
	int maxBlobPoints;
    int maxBlobs;

    ofFbo zonesFbo;
    ofPixels zonesPixels;
    vector< vector<ofPoint> > zones;
    bool zonesOpen;
    void saveZones();
    void loadZones();
    void drawZones();
    
    ofxCvGrayscaleImage zonesMask;
    
    ofFbo mapFbo;
    ofPixels mapPixels;
    ofPoint map[MAP_POINTS];
    int mapPoint;
    bool mapOpen;
    void saveMap();
    void loadMap();
    void drawMap();
    
    ofxCvGrayscaleImage mapMask;
    
    ofxUISuperCanvas *gui;
	void guiEvent(ofxUIEventArgs &e);

};
