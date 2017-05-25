# ar_tracker

## Introduction

The ros-package ar_tracker is a realtime fiducial marker tracker based on ARToolkit 5. It performs robustly even
 under changing illumination conditions. It was written only for the special purpose to
 detect, localize, and track ONE marker (as opposed to track multiple markers in an image frame). The idea
 is to select one marker from an image frame and than simply track this marker in the consecutive ones in a smaller roi 
 and an adaptive image scaling.
  

What is this package for?
- reliably tracking a single fiducial maker
- very high framerates: tested on an 2048x1080 image stream with 35fps at 10% CPU usage on an very old Intel I5
- suitable for outdoor environments with changing illumination conditions due to the adaptive thresholding of ART5
- detecting multiple markers and selecting one out of them for high speed tracking

How is this achieved?
- The algorithm scales the image such that the marker is of constant size in the image e.g. 40 pixel
- Then a roi is defined around the marker
- Each iteration a new roi position is predicted by using the previous marker pose and computing the gradient
- If the maker is lost, the roi grows in size each step till the whole image is again covered

## Installation

- Clone project 
- download ARToolkit 5: https://github.com/artoolkit/artoolkit5
- compile ARToolkit 5 by following the instructions
- create a folder "ARToolkit" in the package root
- copy the folders "lib" and "include" from the artoolkit5 library to the "ARToolkit" folder you just created
- catkin make
- check out the config folder
