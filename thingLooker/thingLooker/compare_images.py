

#methods
# - histograms
# - features 2d

from skimage.metrics import structural_similarity
import cv2
import numpy as np
from sentence_transformers import SentenceTransformer, util
from PIL import Image
import glob
import os

def image_compare_SSIM(img1,img2):

    # Convert images to grayscale
    first_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    second_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    # Compute SSIM between two images
    score, diff = structural_similarity(first_gray, second_gray, full=True)
    print("Similarity Score: {:.3f}%".format(score * 100))

    # The diff image contains the actual image differences between the two images
    # and is represented as a floating point data type so we must convert the array 
    # to 8-bit unsigned integers in the range [0,255] before we can use it with OpenCV
    diff = (diff * 255).astype("uint8")

    # Threshold the difference image, followed by finding contours to
    # obtain the regions that differ between the two images
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    # Highlight differences
    mask = np.zeros(img1.shape, dtype='uint8')
    filled = img2.copy()

    for c in contours:
        area = cv2.contourArea(c)
        if area > 100:

            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(img1, (x, y), (x + w, y + h), (36,255,12), 2)
            cv2.rectangle(img2, (x, y), (x + w, y + h), (36,255,12), 2)
            cv2.drawContours(mask, [c], 0, (0,255,0), -1)
            cv2.drawContours(filled, [c], 0, (0,255,0), -1)

    cv2.imshow('first', img1)
    cv2.imshow('second', img2)
    cv2.imshow('diff', diff)
    cv2.imshow('mask', mask)
    cv2.imshow('filled', filled)
    cv2.waitKey()

def compare_images_DenseV(img1,img2):

    img1 = Image.fromarray(cv2.cvtColor(img1, cv2.COLOR_BGR2RGB))
    img2 = Image.fromarray(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
    # Load the OpenAI CLIP Model
    print('Loading CLIP Model...')
    model = SentenceTransformer('clip-ViT-B-32')

    # Next we compute the embeddings
    # To encode an image, you can use the following code:
    # from PIL import Image
    # encoded_image = model.encode(Image.open(filepath))
    #image_names = list(glob.glob('./*.jpg'))
    #print("Images:", len(image_names))
    encoded_images = model.encode([img1,img2], batch_size=128, convert_to_tensor=True, show_progress_bar=True)

    # Now we run the clustering algorithm. This function compares images aganist 
    # all other images and returns a list with the pairs that have the highest 
    # cosine similarity score
    processed_images = util.paraphrase_mining_embeddings(encoded_images)
    #NUM_SIMILAR_IMAGES = 2

    # =================
    # DUPLICATES
    # =================
    print('Finding duplicate images...')
    # Filter list for duplicates. Results are triplets (score, image_id1, image_id2) and is scorted in decreasing order
    # A duplicate image will have a score of 1.00
    # It may be 0.9999 due to lossy image compression (.jpg)
    score = processed_images[0][0]

    # Output the top X duplicate images
    #score, image_id1, image_id2 = duplicates[0]
    print("\nScore: {:.3f}%".format(score * 100))

    # =================
    # NEAR DUPLICATES
    # =================
    #print('Finding near duplicate images...')
    # Use a threshold parameter to identify two images as similar. By setting the threshold lower, 
    # you will get larger clusters which have less similar images in it. Threshold 0 - 1.00
    # A threshold of 1.00 means the two images are exactly the same. Since we are finding near 
    # duplicate images, we can set it at 0.99 or any number 0 < X < 1.00.
"""    threshold = 0.99
    near_duplicates = [image for image in processed_images if image[0] < threshold]

    for score, image_id1, image_id2 in near_duplicates[0:NUM_SIMILAR_IMAGES]:
        print("\nScore: {:.3f}%".format(score * 100))
        print(image_names[image_id1])
        print(image_names[image_id2])"""

def get_camera():

    # define a video capture object 
    vid = cv2.VideoCapture(0) 

    ret, frame = vid.read() 
    
    # Display the resulting frame 
    cv2.imshow('frame', frame) 
    
    while(True): 
        
        # Capture the video frame 
        # by frame 
        ret, frame = vid.read() 
    
        # Display the resulting frame 
        #cv2.imshow('frame', frame) 
        
        # the 'q' button is set as the 
        # quitting button you may use any 
        # desired button of your choice 

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): 
            break
            
        if key == ord('c'):
            frame1 = frame
            print("frame1 set")

        if key == ord('v'):
            frame2 = frame
            print("frame2 set")
        
        if key == ord('b'):
            compare_and_visualize_differences(frame1,frame2)

    # After the loop release the cap object 
    vid.release() 
    # Destroy all the windows 
    cv2.destroyAllWindows() 

def compare_and_visualize_differences(img1, img2, min_contour_area=100):
    # Convert images to grayscale
    first_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    second_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # Compute SSIM between two images
    score, diff = structural_similarity(first_gray, second_gray, full=True)
    print("Similarity Score: {:.3f}%".format(score * 100))

    # Normalize the difference image to the range [0,255] and convert to uint8
    diff = (diff * 255).astype("uint8")

    # Threshold the difference image, followed by finding contours to obtain the regions of difference
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize mask to highlight differences
    mask = np.zeros(img1.shape, dtype='uint8')
    filled = img2.copy()

    # Loop over the contours
    for c in contours:
        area = cv2.contourArea(c)
        # Only consider contours with area greater than the specified threshold
        if area > min_contour_area:
            x, y, w, h = cv2.boundingRect(c)
            # Draw rectangles around differences
            cv2.rectangle(img1, (x, y), (x + w, y + h), (36, 255, 12), 2)
            cv2.rectangle(img2, (x, y), (x + w, y + h), (36, 255, 12), 2)
            # Fill in the mask and filled images with contours
            cv2.drawContours(mask, [c], 0, (0, 255, 0), -1)
            cv2.drawContours(filled, [c], 0, (0, 255, 0), -1)

    # Display the images
    cv2.imshow('first', img1)
    cv2.imshow('second', img2)
    cv2.imshow('diff', diff)
    cv2.imshow('mask', mask)
    cv2.imshow('filled', filled)
    cv2.waitKey(0)
    #cv2.destroyAllWindows()




get_camera()


