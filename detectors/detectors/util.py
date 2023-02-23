import cv2
import numpy as np

aru_dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
aru_params = cv2.aruco.DetectorParameters_create()


red    = (255,   0,   0)
green  = (  0, 255,   0)
blue   = (  0,   0, 255)
yellow = (255, 255,   0)
white  = (255, 255, 255)

def aruco(frame, drawframe=None):
    # Detect.
    (boxes, ids, rejected) = cv2.aruco.detectMarkers(
        frame, aru_dict, parameters=aru_params)

    # Loop over each marker: the list of corners and ID for this marker.
    box_msg = []
    if len(boxes) > 0:
        for (box, id) in zip(boxes, ids.flatten()):

            # The corners are top-left, top-right, bottom-right,
            # bottom-left of the original marker (not the bounding
            # box).  Each is a 2x1 numpy array of pixel
            # coordinates.  Their type is floating-point though
            # they contain integer values.
            (topLeft, topRight, bottomRight, bottomLeft) = box[0]
            center = (topLeft + bottomRight)/2

            if drawframe is not None:
                # Draw the box around the marker.
                pts = [box.reshape(-1,1,2).astype(int)]
                cv2.polylines(drawframe, pts, True, green, 2)
            
                # Draw the center circle and write the ID there.
                ctr = tuple(center.astype(int))
                cv2.circle(drawframe, ctr, 4, red, -1)
                cv2.putText(drawframe, str(id), ctr,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, green, 2)

            box_msg.append(float(id))
            box_msg.extend(center.flatten().tolist())
            box_msg.extend(topLeft.flatten().tolist())
            box_msg.extend(topRight.flatten().tolist())
            box_msg.extend(bottomRight.flatten().tolist())
            box_msg.extend(bottomLeft.flatten().tolist())
    
    return np.array(box_msg).reshape((-1, 11))



def perspective_transform(data, x0 = -0.6, y0 = 0.0, dx = 0.135, dy = -0.19, ref = [0,4,25,49]):
    
    if not all(np.isin(ref, data[:,0])): return

    ref_data = []
    for i in range(len(ref)):
        data_ind = np.argwhere( data[:,0]==ref[i] )[0,0]
        ref_data.append(data[data_ind])
    pt_from = np.float32([ ref_data[0][1:3].tolist(), ref_data[1][1:3].tolist(), ref_data[2][1:3].tolist(), ref_data[3][1:3].tolist() ])
    pt_to = np.float32([[x0, y0], [x0+dx, y0], [x0, y0+dy], [x0+dx, y0+dy]])
    M = cv2.getPerspectiveTransform(pt_from,pt_to)
    return M

def map2grid(x, y):
    x = x + 60.0
    x_grid = int(x//5 + 6)
    y_grid = int(y//5 + 6)
    return x_grid, y_grid
def grid2map(x_grid, y_grid):
    x = (x_grid-6)*5 + 2.5
    y = (y_grid-6)*5 + 2.5
    x = x-60.0
    return x, y

def piledetect(frame, drawframe=None, M=None, myprint=print, logprob=None, accurate_pos=None):
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Help to determine the HSV range...
    if drawframe is not None:
        # Draw the center lines.  Note the row is the first dimension.
        (H, W, _) = drawframe.shape
        xc = W//2
        yc = H//2
        cv2.line(drawframe, (xc,0), (xc,H-1), white, 1)
        cv2.line(drawframe, (0,yc), (W-1,yc), white, 1)

        # Report the center HSV values.  Note the row comes first.
        # self.get_logger().info(
        # myprint("HSV = (%3d, %3d, %3d)" % tuple(hsv[yc, xc]))

    
    # Threshold in Hmin/max, Smin/max, Vmin/max
    hsvLower = ( 0 ,  0 , 0)
    hsvUpper = (255, 100, 255)
    binary = cv2.inRange(hsv, hsvLower, hsvUpper)

    # Erode and Dilate. Definitely adjust the iterations!
    binary = cv2.erode( binary, None, iterations=1)
    binary = cv2.dilate(binary, None, iterations=2)
    binary = cv2.erode( binary, None, iterations=1)


    # Find contours in the mask and initialize the current
    # (x, y) center of the ball
    (contours, hierarchy) = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # handle probability
    SCALE = 0.02  # smaller means grow slower
    logprob *= (1 - SCALE)
    W, H = logprob.shape
    # Draw all contours on the original image for debugging.
    for cnt in contours:
        epsilon = 0.02*cv2.arcLength(cnt,True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        isPile = len(approx)!=4
        if M is not None:
            projected_cnt = cv2.perspectiveTransform(np.array(approx, dtype='float32'), M)
            ((x, y), (w, h), _) = cv2.minAreaRect(projected_cnt * 1000)
            if w<30 or h<30 or w>200 or h>200: continue
            if not isPile:
                ratio= float(w)/h
                if ratio<0.9 or ratio>1.1:
                    isPile = True

            # log prob process
            center = np.average(approx[:, 0, :], axis=0).astype(int)
            center_perp = cv2.perspectiveTransform(np.array([[center]], dtype='float32'), M)
            x_grid, y_grid = map2grid(center_perp[0,0,0]*100, center_perp[0,0,1]*100)
            
            if 0<=x_grid<W and 0<=y_grid<H:
                accurate_pos[x_grid, y_grid] = [x/1000, y/1000]
                
                if isPile:
                    logprob[x_grid, y_grid] -= SCALE
                else:
                    logprob[x_grid, y_grid] += SCALE
                
                if logprob[x_grid, y_grid]>1: logprob[x_grid, y_grid] = 1.
                elif logprob[x_grid, y_grid]<-1: logprob[x_grid, y_grid] = -1.
            
                # if logprob[x_grid, y_grid] > 0.5 or logprob[x_grid, y_grid] < -0.5:
                #     color = red if logprob[x_grid, y_grid]<0 else blue
                #     radius = int(abs(logprob[x_grid, y_grid])*10)

                #     xx, yy = grid2map(x_grid, y_grid)
                #     center_grid = cv2.perspectiveTransform(np.array([[[ xx/100, yy/100 ]]], dtype='float32'), np.linalg.inv(M))
                #     center_grid = center_grid[0,0].astype(int)
                #     cv2.circle(drawframe, center, radius, color, -1)

                color = int( 255 * abs(logprob[x_grid, y_grid]) )
                if logprob[x_grid, y_grid]>0:
                    cv2.drawContours(drawframe, [approx], -1, (0, 0, color), 2)
                else:
                    cv2.drawContours(drawframe, [approx], -1, (color, 0, 0), 2)
        else:
            if not isPile:
                cv2.drawContours(drawframe, [approx], -1, blue, 2)
            else:
                cv2.drawContours(drawframe, [approx], -1, red, 2)


    
    # Only proceed if at least one contour was found.  You may
    # also want to loop over the contours...
    # if len(contours) > 0:
        # Pick the largest contour.
        # contour = max(contours, key=cv2.contourArea)

        # rect = cv2.minAreaRect(contour)
        # box = cv2.boxPoints(rect)
        # box = np.int0(box)
        # cv2.drawContours(frame,[box],0,(255,0,0),2)

        # Report.
        # self.get_logger().info(
        #     "Found Ball enclosed by radius %d about (%d,%d)" %
        #     (radius, x, y))
def warp_point(M, x: int, y: int):
    d = M[2, 0] * x + M[2, 1] * y + M[2, 2]

    return np.array([
        round((M[0, 0] * x + M[0, 1] * y + M[0, 2]) / d , 4), # x
        round((M[1, 0] * x + M[1, 1] * y + M[1, 2]) / d , 4) # y
    ])