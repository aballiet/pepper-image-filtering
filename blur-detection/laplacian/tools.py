# import the necessary packages
from imutils import paths
import cv2
import shutil

def check_if_blurred(folder, threshold, crop_factor=0, copy=False, logging=False):
    counter = 0
    total   = 0
    
    for image_name in paths.list_images(folder):

        image = cv2.imread(image_name)
        height, width = len(image), len(image[0])

        # cropping image
        image = image[ int(height*crop_factor): int(height*(1-crop_factor)),
                    int(width*crop_factor) : int(width*(1-crop_factor))]

        # convert to black and white
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # compute laplacian variation
        fm = cv2.Laplacian(image, cv2.CV_64F).var()
        
        # if the focus measure is less than the supplied threshold,
        # then the image should be considered "blurry"
        
        if fm < threshold:
            counter += 1
            if copy:
                shutil.copy2(image_name, dest_blurred) # target filename is /dst/dir/file.ext
            
        else:
            if copy :
                shutil.copy2(image_name, dest_sharp) # target filename is /dst/dir/file.ext
         
        total += 1
    
    if logging:
        print("Detected {} blurred images out of {}".format(counter, total))
        print("Blurred image ratio : {}".format(counter / total))
    
    return counter / total