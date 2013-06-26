#include "testApp.h"
#include "homography.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

    ofEnableAlphaBlending();
    ofSetPolyMode(OF_POLY_WINDING_NONZERO);

	backgroundImg.loadImage("mapa.jpg");

    // enable depth->rgb image calibration
	kinect.setRegistration(true);

	//kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	kinect.init(false, false); // disable video image (faster fps)
	kinect.open();

    angle=0;
	//kinect.setCameraTiltAngle(angle);
	//ofSleepMillis(1000);

	kinect.enableDepthNearValueWhite(true);

    ofAddListener(touchTracker.blobAdded, this, &testApp::touchAdded);
    ofAddListener(touchTracker.blobMoved, this, &testApp::touchMoved);
    ofAddListener(touchTracker.blobDeleted, this, &testApp::touchDeleted);

    background.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    backgroundTex.allocate(kinect.width,kinect.height);//,OF_IMAGE_GRAYSCALE);
    thresMask.allocate(kinect.width, kinect.height);
    inPainter.setup(kinect.width, kinect.height);

    tmpThresMask = new unsigned char[kinect.width*kinect.height];
    tmpMapMask = new unsigned char[kinect.width*kinect.height];
    tmpZonesMask = new unsigned char[kinect.width*kinect.height];

    nearThreshold=10000.;
    farThreshold=10000.;

    touchDiffFarThreshold=50.;
    touchDiffNearThreshold=10.;

    maxBlobs = 10;
    minBlobPoints=250;
    maxBlobPoints=1000000;

    dilate=10;
    erode=10;

    gui = new ofxUISuperCanvas("kinectMap", OFX_UI_FONT_MEDIUM);
    gui->addSpacer();
    gui->addTextArea("CONTROL", "Control de parametros de kinectMap");
    gui->addSpacer();
    gui->addSlider("angle", -30, 30, &angle);
    gui->addLabelToggle("learnBackground", &learnBackground);
    gui->addSlider("backFrames", 0.0, BACKGROUND_FRAMES, &backFrames);
    gui->addSpacer();
    gui->addSlider("maxBlobs", 0, 20, &maxBlobs);
    gui->addSlider("min blob points", 0, 2000, &minBlobPoints);
    gui->addSpacer();
    gui->addRangeSlider("near and far threshold", 0., 5000., &nearThreshold,&farThreshold);
    gui->addSpacer();
    gui->addRangeSlider("touch near and far threshold", 0., 50., &touchDiffNearThreshold,&touchDiffFarThreshold);
    gui->addSpacer();
    gui->addLabelToggle("mapOpen", &mapOpen);
    gui->addSpacer();
    gui->addLabelToggle("zonesOpen", &zonesOpen);
    gui->addSpacer();
    gui->addSlider("dilate", 0, 20, &dilate);
    gui->addSlider("erode", 0, 20, &erode);
    gui->autoSizeToFitWidgets();
    ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);

    if(ofFile::doesFileExist("GUI/guiSettings.xml"))
        gui->loadSettings("GUI/guiSettings.xml");

    screenRef[0]=ofPoint(0,0);
	screenRef[1]=ofPoint(640,0);
	screenRef[2]=ofPoint(640,480);
	screenRef[3]=ofPoint(0,480);
    mapPoint=0;
	mapFbo.allocate(kinect.width,kinect.height);
	mapPixels.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    mapMask.allocate(kinect.width, kinect.height);
    loadMap();

	zonesFbo.allocate(kinect.width,kinect.height);
	zonesPixels.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    zonesMask.allocate(kinect.width, kinect.height);
    loadZones();

    sender.setup(IP,PORT);

    mapOpen=false;

    zonesOpen=false;

    learnBackground = true;
    backFrames=0.;

	ofSetFrameRate(60);

}

//--------------------------------------------------------------
void testApp::update() {

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load depth image from the kinect source
		ofFloatPixels current=kinect.getDistancePixelsRef();

        int numPixels = kinect.width*kinect.height;
        unsigned char * tmpCurrent = kinect.getDepthPixels();
        unsigned char * tmpBackground = backgroundTex.getPixels();

        if(learnBackground || backFrames)
        {
            if(backFrames){
                for(int i=0;i<numPixels;i++)
                {
                    if(background[i]){
                        if(current[i]){
                            background[i]=(background[i]+current[i])/2.;
                            unsigned int mean = tmpCurrent[i]+ tmpBackground[i];
                            tmpBackground[i] = (unsigned char)(mean/2);
                        }
                    }
                    else{
                        background[i]=current[i];
                        tmpBackground[i]=tmpCurrent[i];
                    }
                }
                backFrames--;
                if(!backFrames){
                    //inPainter.inpaint(backgroundTex);
                }
            }
            if(learnBackground){
                for(int i=0;i<numPixels;i++)
                {
                    background[i]=current[i];
                    tmpBackground[i]= tmpCurrent[i];
                }
                backFrames=BACKGROUND_FRAMES;
                learnBackground = false;
            }
        }

        float backUpdateFast=0.05;
        float backUpdateSlow=0.005;

        for(int i=0;i<numPixels;i++)
        {
            tmpThresMask[i]=0;
            tmpMapMask[i]=0;
            tmpZonesMask[i]=0;
            float tmpBackDist = background[i];
            float tmpBackImg = tmpBackground[i];
            if(current[i]){
                tmpBackDist = (1.-backUpdateFast)*background[i] + backUpdateFast*current[i];
                tmpBackImg = (1.-backUpdateFast)*((float)tmpBackground[i]) + backUpdateFast*((float)tmpCurrent[i]);
            }
            if(current[i]<farThreshold && current[i]>nearThreshold)
            {
                tmpThresMask[i]=(unsigned char)ofMap(current[i],nearThreshold,farThreshold,255,0);
                if(mapPixels[i]){
                    float diff=background[i]-current[i];
                    if(diff>touchDiffNearThreshold){
                        tmpMapMask[i]=255;
                        tmpBackDist = (1.-backUpdateSlow)*background[i] + backUpdateSlow*current[i];
                        tmpBackImg = (1.-backUpdateSlow)*((float)tmpBackground[i]) + backUpdateSlow*((float)tmpCurrent[i]);
                        if(zonesPixels[i]){
                            if(diff<touchDiffFarThreshold){
                                tmpZonesMask[i]=255;//(unsigned char)ofMap(diff,touchDiffNearThreshold,touchDiffFarThreshold,100,255);
                            }
                        }
                    }
                }
            }
            background[i] = tmpBackDist;
            tmpBackground[i] = (unsigned char)tmpBackImg;
        }
        thresMask.setFromPixels(tmpThresMask, kinect.width, kinect.height);
        mapMask.setFromPixels(tmpMapMask, kinect.width, kinect.height);
        zonesMask.setFromPixels(tmpZonesMask, kinect.width, kinect.height);

        cvErode(mapMask.getCvImage(), mapMask.getCvImage(), NULL, erode);
        cvDilate(mapMask.getCvImage(), mapMask.getCvImage(), NULL, dilate);

        cvAnd(zonesMask.getCvImage(),mapMask.getCvImage(),zonesMask.getCvImage());

        touchTracker.update( zonesMask, -1, minBlobPoints , maxBlobPoints, maxBlobs, 20, false, true);

        backgroundTex.flagImageChanged();

    }

}

//--------------------------------------------------------------
void testApp::draw() {
	if(zonesOpen){
        ofSetColor(255);
        backgroundImg.draw(0, 0, ofGetWidth(), ofGetHeight());
        if(zones.size() && zones.back().size()){
            ofSetColor(255, 255, 255, 150);
            //ofScale(ofGetWidth()/kinect.width,ofGetHeight()/kinect.height);
            ofBeginShape();
            ofCurveVertex(zones.back().front().x*ofGetWidth()/kinect.width,zones.back().front().y*ofGetHeight()/kinect.height);
            ofCurveVertex(zones.back().front().x*ofGetWidth()/kinect.width,zones.back().front().y*ofGetHeight()/kinect.height);
            for(int i=1;i<(zones.back().size()-1);i++){
                ofCurveVertex(zones.back()[i].x*ofGetWidth()/kinect.width,zones.back()[i].y*ofGetHeight()/kinect.height);
            }
            ofCurveVertex(zones.back().back().x*ofGetWidth()/kinect.width,zones.back().back().y*ofGetHeight()/kinect.height);
            ofCurveVertex(zones.back().back().x*ofGetWidth()/kinect.width,zones.back().back().y*ofGetHeight()/kinect.height);
            ofEndShape();
        }
        ofDrawBitmapString("Ready to add points",10,10);
    }
    else if(mapOpen){
        ofSetColor(255);
        thresMask.draw(0, 0, ofGetWidth(), ofGetHeight());
        ofSetColor(255, 0 , 0, 60);
        mapFbo.draw(0, 0, ofGetWidth(), ofGetHeight());
    }
    else{
		ofBackground(100, 100, 100);

		ofSetColor(255, 255, 255);
		// draw from the live kinect
		kinect.drawDepth(10, 10, 320, 240);

		backgroundTex.draw(340,10,320,240);

		thresMask.draw(10, 260, 320, 240);

		ofSetColor(255, 0 , 0);
		mapMask.draw(340, 260, 320, 240);

		ofSetColor(0, 0, 255, 150);
		zonesMask.draw(340, 260, 320, 240);

		ofSetColor(255,0,255,60);
		mapFbo.draw(10,260,320,240);

		ofSetColor(255,255,0,100);
		zonesFbo.draw(10,260,320,240);

		touchTracker.draw(340,260,320,240);

		// draw instructions
		ofSetColor(255, 255, 255);
		stringstream reportStream;
		reportStream << "num touch points found: " << touchTracker.size()
		<< ", fps: " << ofToString(ofGetFrameRate(),2) << endl
		<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
		ofDrawBitmapString(reportStream.str(),20,510);
		if(backFrames)
		{
			ofDrawBitmapString("WAIT!",360,100);
		}
	}
}

//--------------------------------------------------------------
void testApp::touchAdded(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " added" );
    int x=_blob.centroid.x*kinect.width;
    int y=_blob.centroid.y*kinect.height;
    int z=255-zonesPixels[x+y*kinect.width];
    ofxOscMessage m;
    m.setAddress("/play");
    m.addIntArg(z);
    sender.sendMessage(m);
}

//--------------------------------------------------------------
void testApp::touchMoved(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " moved" );
}

//--------------------------------------------------------------
void testApp::touchDeleted(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " deleted" );
}

//--------------------------------------------------------------
void testApp::guiEvent(ofxUIEventArgs &e)
{
    string name = e.widget->getName();
    if(name=="angle")
    {
        if(angle>30.) angle=30.;
        if(angle<-30.) angle=-30.;
        kinect.setCameraTiltAngle(angle);
    }
    else if(name=="zonesOpen"){
        if(zonesOpen){
            vector<ofPoint> points;
            zones.push_back(points);
			ofSetFullscreen(true);
        }
        else{
			ofSetFullscreen(false);
            if(zones.size()){
                if(zones.back().size()){
                    for(int i=0;i<zones.back().size();i++){
                        zones.back()[i]=homography*zones.back()[i];
                    }
                }
				else{
                    zones.pop_back();
				}
                drawZones();
            }
        }
    }
}

//--------------------------------------------------------------
void testApp::exit(){
    //kinect.setCameraTiltAngle(0);
	kinect.close();

    gui->saveSettings("GUI/guiSettings.xml");

    saveZones();

    saveMap();

    delete tmpThresMask;
    delete tmpMapMask;
    delete tmpZonesMask;

    delete gui;
}

//--------------------------------------------------------------
void testApp::loadZones(){
    ofDirectory dir;
    dir.allowExt("zone");
    dir.listDir("zones/");
	dir.sort(); // in linux the file system doesn't return file lists ordered in alphabetical order

	// you can now iterate through the files and load them into the ofImage vector
	for(int i = 0; i < (int)dir.size(); i++){
        ofBuffer buf=ofBufferFromFile(dir.getPath(i));
        vector<ofPoint> points;
        //Read file line by line
        while (!buf.isLastLine()) {
            string line = buf.getNextLine();
            //Split line into strings
            vector<string> p = ofSplitString(line, ",");
            points.push_back(ofPoint(ofToInt(p[0]),ofToInt(p[1])));
        }
        if(points.size())
            zones.push_back(points);
	}
    drawZones();
}

//--------------------------------------------------------------
void testApp::saveZones(){
    int f=0;
    for(int i=0;i<zones.size();i++){
        if(zones[i].size()){
            ofBuffer buf;
            for(int j=0;j<zones[i].size();j++){
                buf.append(ofToString(zones[i][j])+"\n");
            }
            ofBufferToFile("zones/"+ofToString(f++)+".zone",buf);
        }
    }
}

//--------------------------------------------------------------
void testApp::loadMap(){
    if(ofFile::doesFileExist("0.map")){
        ofBuffer buf=ofBufferFromFile("0.map");
        int i=0;
        //Read file line by line
        while (!buf.isLastLine() && i<MAP_POINTS) {
            string line = buf.getNextLine();
            //Split line into strings
            vector<string> p = ofSplitString(line, ",");
            map[i++]=ofPoint(ofToInt(p[0]),ofToInt(p[1]));
        }
	}
	homography=findHomography(screenRef,map);
    drawMap();
}

//--------------------------------------------------------------
void testApp::saveMap(){
    ofBuffer buf;
    for(int i=0;i<MAP_POINTS;i++){
        buf.append(ofToString(map[i])+"\n");
    }
    ofBufferToFile("0.map",buf);
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
        case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
    }

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    ofxUIRectangle * guiWindow = gui->getRect();
    if(!guiWindow->inside(x,y)){//mouseX>10 && mouseX<340 && mouseY>260 && mouseY<500){
        int mX=x*kinect.width/ofGetWidth();//2*(mouseX-10);
        int mY=y*kinect.height/ofGetHeight();;//2*(mouseY-260);
        if(zonesOpen){
            if(button==0){
                zones.back().push_back(ofPoint(mX,mY));
            }
            else if(button==2){
                if(zonesPixels[mX+mY*kinect.width]){
                    int index=255-zonesPixels[mX+mY*kinect.width];
                    vector< vector<ofPoint> >::iterator zoneIterator;
                    zoneIterator=zones.begin();
                    for(int i=0;i<index;i++)
                        zoneIterator++;
                    zones.erase(zoneIterator);
                }
            }
            drawZones();
        }
        else if(mapOpen){
            map[mapPoint++]=ofPoint(mX,mY);
            if(mapPoint>=MAP_POINTS){
                mapOpen=false;
                mapPoint=0;
            }
            homography=findHomography(screenRef,map);
            drawMap();
        }
    }
}

//--------------------------------------------------------------
void testApp::drawZones()
{
    zonesFbo.begin();
    ofClear(0,0);
    for(int i=0;i<zones.size();i++){
        if(zones[i].size()==1)
        {
            ofSetColor(255-i);
            ofPoint p0=zones[i].front();
            ofLine(p0.x,p0.y,p0.x,p0.y);
        }
        if(zones[i].size()==2)
        {
            ofSetColor(255-i);
            ofPoint p0=zones[i].front();
            ofPoint p1=zones[i].back();
            ofLine(p0.x,p0.y,p1.x,p1.y);
            zonesFbo.end();

        }
        if(zones[i].size()>=3)
        {
            ofSetColor(255-i);
            ofPoint p0=zones[i].front();
            ofBeginShape();
            ofCurveVertex(p0.x,p0.y);
            ofCurveVertex(p0.x,p0.y);
            for(int j=1;j<(zones[i].size()-1);j++)
            {
                ofPoint p=zones[i][j];
                ofCurveVertex(p.x,p.y);
            }
            ofPoint p1=zones[i].back();
            ofCurveVertex(p1.x,p1.y);
            ofCurveVertex(p1.x,p1.y);
            ofEndShape();
        }
    }
    zonesFbo.end();
    ofPixels tmpPixels;
    zonesFbo.readToPixels(tmpPixels);
    for(int i=0;i<zonesPixels.getWidth()*zonesPixels.getHeight();i++){
        zonesPixels[i]=tmpPixels[i*4];
    }
}

//--------------------------------------------------------------
void testApp::drawMap()
{
    mapFbo.begin();
    ofClear(0,0);
    ofSetColor(255);
    ofBeginShape();
    ofVertex(map[0].x,map[0].y);
    ofVertex(map[1].x,map[1].y);
    ofVertex(map[2].x,map[2].y);
    ofVertex(map[3].x,map[3].y);
    ofEndShape();
    mapFbo.end();
    ofPixels tmpPixels;
    mapFbo.readToPixels(tmpPixels);
    for(int i=0;i<mapPixels.getWidth()*mapPixels.getHeight();i++){
        mapPixels[i]=tmpPixels[i*4];
    }
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
}
