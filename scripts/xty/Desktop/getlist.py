#!/usr/bin/python
# -*- coding:UTF-8 -*-
import cv2
import numpy as np
img_path = "1.jpg"

def show_contours(img):
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
    
    image,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    contours = contours[1:]
    #cv2.drawContours(img,contours,-1,(255,0,255),1)
    #cv2.imshow('img',img)
    cv2.waitKey(0)

    return contours

if __name__=="__main__":
    img = cv2.imread(img_path)
    show_contours(img)