from skimage.metrics import structural_similarity
import cv2
import numpy as np
from sentence_transformers import SentenceTransformer, util
from PIL import Image
import glob
import os
import matplotlib.pyplot as plt

import skimage


def image_compare_SSIM(img1,img2):
    """
    A Basic Image comparison approach which directly compares images to find regions of difference. 
    Thresholding is implemented so that difference finding is adjustable. Not robust to small turns.
    """

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
    """
    Another image comparison implementation. This implementation uses sentence transformers to 
    compare images and does not produce visual output but is more robust to small turns.
    """

    img1 = Image.fromarray(cv2.cvtColor(img1, cv2.COLOR_BGR2RGB))
    img2 = Image.fromarray(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
    # Load the OpenAI CLIP Model
    print('Loading CLIP Model...')
    model = SentenceTransformer('clip-ViT-B-32')

    # Next we compute the embeddings
    encoded_images = model.encode([img1,img2], batch_size=128, convert_to_tensor=True, show_progress_bar=True)

    # Now we run the clustering algorithm. This function compares images aganist 
    # all other images and returns a list with the pairs that have the highest 
    # cosine similarity score
    processed_images = util.paraphrase_mining_embeddings(encoded_images)
    print('Finding duplicate images...')
    # Filter list for duplicates. Results are triplets (score, image_id1, image_id2) and is scorted in decreasing order
    # A duplicate image will have a score of 1.00
    # It may be 0.9999 due to lossy image compression (.jpg)
    score = processed_images[0][0]

    # Output the top X duplicate images
    #score, image_id1, image_id2 = duplicates[0]
    print("\nScore: {:.3f}%".format(score * 100))


def get_camera():
    """
    A script that tests image comparison by using the laptop webcam.
    """

    # define a video capture object 
    vid = cv2.VideoCapture(0) 

    ret, frame = vid.read() 
    print("Got frame")
    # Display the resulting frame 
    #cv2.imshow('frame', frame) 
    
    while(True): 
        
        # Capture the video frame 
        # by frame 
        ret, frame = vid.read() 

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

    # After the loop release the cap objectq
    vid.release() 
    # Destroy all the windows 
    cv2.destroyAllWindows()

def compare_and_visualize_differences(img1, img2, min_contour_area=100):
    """
    A Basic Image comparison approach which directly compares images to find regions of difference. 
    Thresholding is implemented so that difference finding is adjustable. Not robust to small turns.

    A version of SSIM similarity comparison with a thresholding on the minimum size of difference to detect.
    """
    
    #img1,img2 = replace_blurry_regions(img1,img2,blur_threshold=65)
    
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
    print("DISPLAYING IMAGES")
    cv2.imshow('first', img1)
    cv2.imshow('second', img2)
    cv2.imshow('diff', diff)
    cv2.imshow('mask', mask)
    cv2.imshow('filled', filled)
    cv2.waitKey(0)
    #cv2.destroyAllWindows()

#blur functions

def detect_blurry_regions(image, threshold=100):
    """
    Detect blurry regions in an image using the Laplacian method.
    Returns a mask indicating blurry regions.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    laplacian_var = laplacian.var()

    # If variance is less than the threshold, it's considered blurry
    mask = np.where(laplacian_var < threshold, 0, 1).astype('uint8')
    
    
    visualize = True
    if visualize:
        plt.figure(figsize=(10, 4))
        plt.subplot(1, 2, 1)
        plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        plt.title('Original Image')
        plt.axis('off')
        plt.subplot(1, 2, 2)
        plt.imshow(laplacian, cmap='gray')
        plt.title('Laplacian Gradient')
        plt.axis('off')
        plt.show()
    
    return mask

def replace_blurry_regions(img1, img2, blur_threshold=100):
    """
    Detects blurry regions in both images and replaces those regions with black pixels in both images.
    Deals with ill defined regions of a NeRF.
    """
    # Detect blurry regions in both images
    mask1 = detect_blurry_regions(img1, blur_threshold)
    mask2 = detect_blurry_regions(img2, blur_threshold)

    # Combine masks to identify all blurry regions in both images
    combined_mask = np.bitwise_or(mask1, mask2)

    # Apply combined mask to both images
    img1[combined_mask == 0] = [0, 0, 0]
    img2[combined_mask == 0] = [0, 0, 0]

    return img1, img2

def get_image_size(img):
    """
    Return the size (width, height) of the image at the given path.

    :param image_path: Path to the image.
    :return: A tuple (width, height).
    """
    #img = cv2.imread(image_path)
    if isinstance(img, np.ndarray):
        return img.shape[1], img.shape[0]
    else:
        return img.size[1], img.size[0]  # Width, Height

def resize_image(img, size):
    """
    Resize an image to the given size and save it to the output path.

    :param input_path: Path to the input image.
    :param output_path: Path to save the resized image.
    :param size: A tuple (width, height) for the target size.
    """
    resized_img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)

    return resized_img
    

