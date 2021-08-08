#!/usr/bin/env python

import cv2
import numpy as np

def hdr_merge(images, exposures, gamma=1.8):
    '''Merge multiple exposure-bracketed images into a single tonemapped image.
    Remove gamma from the images prior to registration.'''
    # NOTE(Jordan): Pitcam applies a 1.8 gamma to its images. Remove it.
    images = [_adjust_gamma(im, gamma) for im in images]
    # Align the images and crop them to the overlapping region.
    images, exposures = _align_image_bracket(images, exposures)
    # Merge the images.
    hdr_image = _merge_images_mertens(images, exposures)
    return hdr_image

def static_vars(**kwargs):
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate

class CachedSIFT:
    '''This thing finds SIFT keypoints and descriptors.
    It caches inputs to avoid re-SIFT-ing when possible.'''
    def __init__(self, capacity):
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.cache_keys = []
        self.cache_vals = []
        self.cache_capacity = capacity

    def detect_and_compute(self, img):
        # Use a hash of the image as a key for the cache.
        key = self._image_hash(img)
        if key in self.cache_keys:
            # If the key is in the cache, return the corresponding value.
            return self.cache_vals[self.cache_keys.index(key)]
        else:
            # Otherwise, compute the SIFT result,
            res = self.sift.detectAndCompute(img, None)
            # and add it to the cache.
            self.cache_keys.append(key)
            self.cache_vals.append(res)
            # If the cache has exceeded its capacity, toss the oldest entry.
            if len(self.cache_keys) > self.cache_capacity:
                self.cache_keys = self.cache_keys[1:]
                self.cache_vals = self.cache_vals[1:]
            # Return the SIFT result.
            return res

    def _image_hash(self, img):
        u = img.view('u' + str(img.itemsize))
        return np.bitwise_xor.reduce(u.ravel())

@static_vars(sift = CachedSIFT(2))
def _align_images_sift(im1, im2):
  '''Align im1 to im2. Warp im1 to match im2.
  Return the warped im1 and the homography h that was used to warp it.'''

  # Convert images to grayscale
  im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
  im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)

  # find the keypoints and descriptors with SIFT
  kp1, des1 = _align_images_sift.sift.detect_and_compute(im1Gray)
  kp2, des2 = _align_images_sift.sift.detect_and_compute(im2Gray)

  # Matching descriptor vectors with a FLANN based matcher
  matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
  matches = matcher.knnMatch(des1, des2, 2)

  # Apply ratio test
  good = []
  for m,n in matches:
      if m.distance < 0.7*n.distance:
          good.append([m])

  # Extract location of good matches
  points1 = np.array([kp1[g[0].queryIdx].pt for g in good])
  points2 = np.array([kp2[g[0].trainIdx].pt for g in good])

  # Find homography
  h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)

  # Use homography
  height, width, channels = im2.shape
  im1Reg = cv2.warpPerspective(im1, h, (width, height))

  return im1Reg, h


def _align_image_bracket(images, exposures):
    '''Sort images by increasing exposure. Align each image to the center image
    in the exposure bracket. Warp and crop all images to cover only the overlapping area.'''

    assert len(images) == len(exposures), 'Please provide one exposure value per image.'

    # N is the number of images to be processed.
    N = len(images)

    # Sort images by increasing exposure.
    image_list = [(images[i], exposures[i]) for i in range(N)]
    image_list.sort(key=lambda e: e[1], reverse=True)

    # Each image will be matched against the image in the center of the exposure bracket.
    base_image = image_list[N/2][0]
    base_exposure = image_list[N/2][1]
    
    # Match images.
    aligned_images = []
    aligned_image_exposures = []
    aligned_image_homographies = []
    for i in range(N):
        curr_image = image_list[i][0]
        curr_exposure = image_list[i][1]

        # Don't match the center image to itself.
        if i == N/2:
            reg_image = curr_image
            H = np.identity(3)
        else:
            base_image = base_image.astype('uint8')
            reg_image, H = _align_images_sift(curr_image, base_image)
        aligned_images.append(reg_image)
        aligned_image_exposures.append(curr_exposure)
        aligned_image_homographies.append(H)

    # Find the bounds of the overlapping area shared by all images.
    bound_x = [0, base_image.shape[1]]
    bound_y = [0, base_image.shape[0]]
    for h in aligned_image_homographies:
        cx = h[0,2]
        cy = h[1,2]
        bound_x[0] = max(bound_x[0], bound_x[0]+cx)
        bound_x[1] = min(bound_x[1], bound_x[1]+cx)
        bound_y[0] = max(bound_y[0], bound_y[0]+cy)
        bound_y[1] = min(bound_y[1], bound_y[1]+cy)
    bound_x = [int(x) for x in bound_x]
    bound_y = [int(y) for y in bound_y]

    # Crop all images to only the overlap.
    aligned_images = [img[bound_y[0]:bound_y[1], bound_x[0]:bound_x[1]] for img in aligned_images]

    return aligned_images, aligned_image_exposures

def _merge_images_mertens(images, exposures):
    # Exposure fusion using Mertens
    merge_mertens = cv2.createMergeMertens()
    res_mertens = merge_mertens.process(images)

    res_mertens_8bit = np.clip(res_mertens*255, 0, 255).astype('uint8')
    res_mertens_8bit = _adjust_gamma(res_mertens_8bit, 1.0/2.2)
    return res_mertens_8bit

def _adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
            for i in np.arange(0, 256)]).astype("uint8")
    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)

if __name__=="__main__":
    image_paths = ['images/ae_-2.jpg', 'images/ae_+0.jpg', 'images/ae_+2.jpg']
    exposures = [5500.0, 11000.0, 44000.0]

    images = [cv2.imread(p) for p in image_paths]
    hdr_image = hdr_merge(images, exposures)
    cv2.imwrite('hdr_merge.jpg', hdr_image)
