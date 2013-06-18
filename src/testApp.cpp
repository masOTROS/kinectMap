#include "testApp.h"
//--------------------------------------------------------------
void testApp::setup() {

	ofSetLogLevel(OF_LOG_VERBOSE);
    
    ofEnableAlphaBlending();
    ofSetPolyMode(OF_POLY_WINDING_NONZERO);

    // enable depth->rgb image calibration
	kinect.setRegistration(true);

	//kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	kinect.init(false, false); // disable video image (faster fps)
	kinect.open();

    angle=0;
	kinect.setCameraTiltAngle(angle);
	//ofSleepMillis(1000);

	kinect.enableDepthNearValueWhite(true);
    
    ofAddListener(touchTracker.blobAdded, this, &testApp::touchAdded);
    ofAddListener(touchTracker.blobMoved, this, &testApp::touchMoved);
    ofAddListener(touchTracker.blobDeleted, this, &testApp::touchDeleted);

    background.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    backgroundTex.allocate(kinect.width,kinect.height);//,OF_IMAGE_GRAYSCALE);
    thresMask.allocate(kinect.width, kinect.height);
    inPainter.setup(kinect.width, kinect.height);

    nearThreshold=560.;
    farThreshold=880.;

    touchDiffFarThreshold=120.;
    touchDiffNearThreshold=7.;

    maxBlobs = 10;
    minBlobPoints=0;
    maxBlobPoints=1000000;
    
    dilate=10;
    erode=4;

	learnBackground = true;
	backFrames=0.;

	zonesOpen=false;
	zonesFbo.allocate(kinect.width,kinect.height);
	zonesPixels.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    loadZones();
    mapMask.allocate(kinect.width, kinect.height);
    
    mapOpen=false;
    mapPoint=0;
	mapFbo.allocate(kinect.width,kinect.height);
	mapPixels.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    loadMap();
    zonesMask.allocate(kinect.width, kinect.height);
    
    gui = new ofxUISuperCanvas("kinectMap", OFX_UI_FONT_MEDIUM);
    gui->addSpacer();
    gui->addTextArea("CONTROL", "Control de parametros de kinectMap");
    gui->addSpacer();
    gui->addSlider("angle", -30., 30., &angle);
    gui->addLabelToggle("learnBackground", &learnBackground);
    gui->addSlider("backFrames", 0.0, BACKGROUND_FRAMES, &backFrames);
    gui->addSpacer();
    gui->addSlider("maxBlobs", 0, 20, &maxBlobs);
    gui->addRangeSlider("min and max blob points", 0, 150000, &minBlobPoints,&maxBlobPoints);
    gui->addSpacer();
    gui->addRangeSlider("near and far threshold", 0., 2000., &nearThreshold,&farThreshold);
    gui->addSpacer();
    gui->addRangeSlider("touch near and far threshold", 0., 500., &touchDiffNearThreshold,&touchDiffFarThreshold);
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

        if(learnBackground || backFrames)
        {
            unsigned char * tmpCurrent = kinect.getDepthPixels();
            unsigned char * tmpBackground = backgroundTex.getPixels();
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
                    inPainter.inpaint(backgroundTex);
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
            backgroundTex.flagImageChanged();
        }
        
        unsigned char * tmpThresMask = new unsigned char[numPixels];
        unsigned char * tmpMapMask = new unsigned char[numPixels];
        unsigned char * tmpZonesMask = new unsigned char[numPixels];
        
        for(int i=0;i<numPixels;i++)
        {
            tmpThresMask[i]=0;
            tmpMapMask[i]=0;
            if(current[i]<farThreshold && current[i]>nearThreshold)
            {
                tmpThresMask[i]=(unsigned char)ofMap(current[i],nearThreshold,farThreshold,255,0);
                if(mapPixels[i]){
                    float diff=background[i]-current[i];
                    if(diff>touchDiffNearThreshold){
                        tmpMapMask[i]=255;
                        if(zonesPixels[i]){
                            if(diff<touchDiffFarThreshold){
                                tmpZonesMask[i]=255;//(unsigned char)ofMap(diff,touchDiffNearThreshold,touchDiffFarThreshold,100,255);
                            }
                        }
                    }
                }
            }
        }
        thresMask.setFromPixels(tmpThresMask, kinect.width, kinect.height);
        mapMask.setFromPixels(tmpMapMask, kinect.width, kinect.height);
        zonesMask.setFromPixels(tmpZonesMask, kinect.width, kinect.height);

        cvErode(mapMask.getCvImage(), mapMask.getCvImage(), NULL, erode);
        cvDilate(mapMask.getCvImage(), mapMask.getCvImage(), NULL, dilate);
        
        cvAnd(zonesMask.getCvImage(),mapMask.getCvImage(),zonesMask.getCvImage());
    
        touchTracker.update( zonesMask, -1, minBlobPoints , maxBlobPoints, maxBlobs, 20, false, true);
    }

}

//--------------------------------------------------------------
void testApp::draw() {
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
    if(zonesOpen)
    {
        ofDrawBitmapString("Ready to add points",10,270);
    }
}

//--------------------------------------------------------------
void testApp::touchAdded(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " added" );
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
        }
        else{
            if(zones.size()){
                if(!zones.back().size()){
                    zones.pop_back();
                }
                drawZones();
            }
        }
    }
}

//--------------------------------------------------------------
void testApp::exit(){
    kinect.setCameraTiltAngle(0);
	kinect.close();
    
    gui->saveSettings("GUI/guiSettings.xml");
    
    saveZones();
    
    saveMap();
    
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
        
		zones.push_back(points);
	}
    drawZones();
}

//--------------------------------------------------------------
void testApp::saveZones(){
    for(int i=0;i<zones.size();i++){
        ofBuffer buf;
        for(int j=0;j<zones[i].size();j++){
            buf.append(ofToString(zones[i][j])+"\n");
        }
        ofBufferToFile("zones/"+ofToString(i)+".zone",buf);
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
    if(mouseX>10 && mouseX<340 && mouseY>260 && mouseY<500){
        int mX=2*(mouseX-10);
        int mY=2*(mouseY-260);
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
