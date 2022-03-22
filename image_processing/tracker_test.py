import cv2
import sys
 
if __name__ == '__main__' :
 
    # Set up tracker.
    # Instead of MIL, you can also use
    # BOOSTING, KCF, TLD, MEDIANFLOW or GOTURN
    
    # Notes - pink ball test
    # BOOSTING - somewhat slow, doesn't deal well with obstruction 
    # KCF - tracks well - deals with obstruction, pretty fast
    # TLD - doesn't pick up on pink ball
    # MEDIANFLOW - pretty fast; fails with obstruction
    # GOTURN - crashes python; some bug in implementation

    # Notes - checkered ball test
    # BOOSTING - tracks ball okay
    # KCF - tracks ball very fast
    # TLD - slow, tracks checkered ball somewhat accurately; loses the ball occasionally
    # MEDIANFLOW - tracks well until partial occlusion; completely loses the ball afterwards
    # GOTURN - crashese; bug in implementation

    tracker = cv2.Tracker_create("KCF")
 
    # Read video
    #"C:/Users/joe/Videos/ballTrackerDemoVideo.mp4"
    #'C:/Users/joe/Documents/Python/pink_ball_tracking_test_video.avi'
    video = cv2.VideoCapture("C:/Users/joe/Documents/Github/foosball/Test Videos/blue ballTrim.mp4")
 
    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        sys.exit()
 
    # Read first frame.
    ok, frame = video.read()
    height, width = frame.shape[:2]
    if not ok:
        print('Cannot read video file')
        sys.exit()
     
    # Define an initial bounding box
    #bbox = (287, 23, 86, 320)
 
    # Uncomment the line below to select a different bounding box
    frame = cv2.resize(frame,(int(width/4),int(height/4)))
    bbox = cv2.selectROI(frame, False)
 
    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
 
    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break
        
        frame = cv2.resize(frame,(int(width/4),int(height/4))) 
        # Update tracker
        ok, bbox = tracker.update(frame)
 
        # Draw bounding box
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0,0,255))
 
        # Display result
        cv2.imshow("Tracking", frame)
 
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
