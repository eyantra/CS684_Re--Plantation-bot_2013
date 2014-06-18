
clc;
getd('./functions')                    //adding the functions directly. location of all my function files
h = openserial(4,"9600,n,8,1"); // open serial port 
//avicloseall();                                                              //closing all open camera ports
//n = camopen(0);                                                            //opening the camera

//for i=1:20                                                                  //looping to provide enough time for camera to initialize before capturing image
//image0 = avireadframe(n);                                                 //reading image from camera

//image0 = imread('real.jpg');
//imshow(image0);
//end

//imshow(image0);
                                                              //closing the open camera port
//h=openserial(4,"9600,n,8,1");
detected=0;
while(detected==0)
    avicloseall();
    //disp(detected);
    //detected = 0;
    n=camopen(0);
    for i=1:10
        im = avireadframe(n);
        imshow(im);
    end;
    //imread('real.jpg');
    imshow(im);
    //avilistopened();
    avicloseall();
    il=imwrite(im,"catch.jpg")
    image0=im;
    imshow(image0);

    orig_image = image0
    imagegray = rgb2gray(image0);                                               //converting color image to grayscale

    //imshow(imagegray);

    i2 = uint8(255*ones(size(imagegray,1),size(imagegray,2))); 

    inverted_image = i2-uint8(imagegray);                                       //complementing the grayscale image
    //imshow(inverted_image);

    //imshow(i2);

    i1 = inverted_image;
    threshold = CalculateOtsuThreshold(i1);                                     //calculating threshold for the given image
    //imshow(threshold);
    disp(threshold,'threshold');
    seg_image = SegmentByThreshold(i1,threshold+20);                               //segmenting the image based on the threshold calculated above

    imshow(seg_image);

    blobimage = SearchBlobs(seg_image);                                         //searching segments for blobs
//imshow(blobimage);

//[CumulatedSizeHistogram Listofsizes] = CumulatedSizeHistogram(blobimage);
//figure();plot(Listofsizes,CumulatedSizeHistogram);


    filteredimage = FilterBySize(blobimage,40);                                    //filtering out blobs that are too small. 
    figure();ShowImage(filteredimage,'image is filtered',jetcolormap(4))

//Iscalculated = CreateFeatureStruct();
    Iscalculated = CreateFeatureStruct(%f);
//Iscalculated.BoundingBox = %t;
    Iscalculated.Centroid = %t;

    Blobstatistics = AnalyzeBlobs(filteredimage,Iscalculated);

    Figurewindow = ShowColorImage(image0,'image with boxes');
    plot(Blobstatistics(1).Centroid(1),Blobstatistics(1).Centroid(2),'r*');
//disp("color",getcolor(image0);
//DrawBoundingBoxes(Blobstatistics,[0 0.5 0],Figurewindow);
    disp("centroid",Blobstatistics(1).Centroid(1),Blobstatistics(1).Centroid(2));
    if Blobstatistics(1).Centroid(1) > 121 then
        detected =1;
    end
//h=openserial(4,"9600,n,8,1");
    if detected == 0 then
        writeserial(h,"8");      // sending command to move bot 2 cm
    else 
        writeserial(h,"1");       //sending command that bot is detected now collect the plant
    end

//xpause(200000);
//buf = readserial(h);
//xpause(200000);

//disp(buf);
//mprintf("buffer %d",buf);

    disp(detected);
end
closeserial(h);
