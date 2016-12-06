import cv2
import os
import numpy as np

mainPath = os.getcwd()
os.chdir("sourcePictures")
im_src = cv2.imread("ironManFront.png")
pts_src = np.array([[0,0],[152,0],[152,255],[0,255]])
im_dst = cv2.imread("Design.png")
pts_dst = np.array([[50,70],[176,100],[176,190],[50,180]])
os.chdir(mainPath)

h, status = cv2.findHomography(pts_src, pts_dst)
print(im_dst.shape[1],im_dst.shape[0])
im_out = cv2.warpPerspective(im_src, h, (im_dst.shape[1],im_dst.shape[0]))

# putting it on im_dst
img1 = im_dst
img2 = im_out
# ROI
rows,cols,channels = img2.shape
roi = img1[0:rows, 0:cols]
# Now create a mask of logo and create its inverse mask also
img2gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
mask_inv = cv2.bitwise_not(mask)

# Now black-out the area of logo in ROI
img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)

# Take only region of logo from logo image.
img2_fg = cv2.bitwise_and(img2,img2,mask = mask)

# Put logo in ROI and modify the main image
dst = cv2.add(img1_bg,img2_fg)
img1[0:rows, 0:cols ] = dst
cv2.imwrite(mainPath+"/ggg.png", img1)
cv2.imshow('res',img1)
cv2.waitKey(0)
cv2.destroyAllWindows()




