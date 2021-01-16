#!/usr/bin/env python

#Import Python libraries
import cv2
import os
import math
from skimage.draw import polygon
import numpy as np
import networkx as nx
from scipy.spatial import distance_matrix
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import sys
import threading
import math

image = cv2.imread("/home/deb/Desktop/seg_4_crop.png")

cv2.imshow("Image", image)
cv2.waitKey(0)
img_height, img_width, _ = image.shape

print("Image shape is {}.".format(image.shape))
print(image)

# calculate difference matrices
row_diff = np.diff(image, axis=0)
col_diff = np.diff(image, axis=1)

#print("col_diff has shape {} and looks like: {}".format(col_diff.shape,col_diff))

# initialize distance matrices
row_dist = np.zeros( (row_diff.shape[0],row_diff.shape[1]) ) 
col_dist = np.zeros( (col_diff.shape[0],col_diff.shape[1]) )

# build distance matrices
for i in range(0,row_diff.shape[0]):
    for j in range(0,row_diff.shape[1]):
        row_dist[i,j] = math.sqrt(np.sum(np.square(row_diff[i,j])))

for i in range(0,col_diff.shape[0]):
    for j in range(0,col_diff.shape[1]):
        col_dist[i,j] = math.sqrt(np.sum(np.square(col_diff[i,j])))

print("Distance matrix for columns: {}".format(col_dist))

eps = 15 # max. distance between pixels to be considered 'close'

gray_image = cv2.cvtColor(np.array(image), cv2.COLOR_BGR2GRAY) # convert to grayscale
_, contours, _ = cv2.findContours( gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE ) # find all contours

#Debugging
#print("Found %d objects in this frame - may or may not all be sherds." % (len(contours)))

# exclude boxes smaller than a minimum area
use_min_area = False
min_area = 0.0006  # sq. meters (roughly 1 sq. inch)
#print ("Recognizing only rectangles larger than %f sq. meters as sherds" % (min_area) )

"""
# Construct Detection3DRPYArray custom ROS message to contain all valid bounding boxes
detections = Detection3DRPYArray()
detections.header = header    # meta-data
"""

# For each detected contour, find bounding box
for cnt in contours:
    rect = cv2.minAreaRect(cnt) # [(x center, y center), (width, height), (rotation) in pixels]
    box = cv2.boxPoints(rect) # [4x1 array of tuples: pixel coordinates of vertices]
    box = np.int0(box)
    #print("Box: {}.".format(box))

    # check that contour does not meet edges of image frame
    in_frame = True
    for corner in box:
        if (corner[0] <= 0) or (corner[0] >= img_width-1) or (corner[1] <= 0) or (corner[1] >= img_height-1):
            in_frame = False
        if not in_frame:
            break
    
    sherd_contours = cv2.drawContours( np.array(image), [box], 0, (255,0,0), 3 )
    plt.figure("Figure 2")
    plt.imshow(sherd_contours)
    plt.title("Bounding Box around Sherd")
    plt.show()

    r = np.array( [vertex[0] for vertex in box] ) # row coordinates of vertices of polygon
    c = np.array( [vertex[1] for vertex in box] ) # column coordinates of vertices of polygon
    rr, cc = polygon(r,c) # row and column coordinates of all pixels inside bounding box
    
    sherd_pile = zip(rr,cc) # merge into one list of tuple coordinates

    # build graph first: add edges and nodes with pixel coord labels??
    # https://networkx.org/documentation/networkx-2.1/reference/classes/generated/networkx.Graph.add_node.html

    G = nx.Graph()
    G.add_nodes_from(sherd_pile) # create node for each pixel in sherd_pile
    print("Done adding nodes from sherd_pile. Length of sherd_pile is {} and number of nodes in graph is {}.".format( len(sherd_pile), G.number_of_nodes() ))

    # constrain i,j to sherd pile (so if pixel coordinate is found in sherd_pile...)
    # [0,0] of dist matrix in row_dist is distance between [0,0] and [1,0] in image.
    # [i,j] of dist matrix in row_dist is distance between [i,j] and [i+1, j] in image
    # so if row_dist[i,j] <= eps, add edge([i,j], [i+1, j])
    # [0,0] of dist matrix in col_dist is distance between [0,0] and [0,1] in image.
    # [i,j] of dist matrix in row_dist is distance between [i,j] and [i, j+1] in image

    for pixel in sherd_pile:
        i,j = pixel[0],pixel[1]
        try:
            if row_dist[i,j] <= eps:
                G.add_edge( (i,j), (i+1,j) )
            elif col_dist[i,j] <= eps:
                G.add_edge( (i,j), (i,j+1) )
        except IndexError:
            pass

    print("Added {} edges between nodes in sherd_pile.".format( G.number_of_edges() ))

    # drawing and showing the nodes can take a prohibitively long time
    #nx.draw(G)
    #plt.show()

    """
    sherd_candidates = [ pixel for pixel in nx.connected_components(G) ]
    print("Sherd candidates as lists of connected pixels: {}".format(sherd_candidates) )
    """

    # next: generate adjacency matrix: https://networkx.org/documentation/networkx-1.9/reference/generated/networkx.linalg.graphmatrix.adjacency_matrix.html
    # get groups of pixels that make up sherd candidates with https://docs.scipy.org/doc/scipy/reference/generated/scipy.sparse.csgraph.connected_components.html
    # figure out what the hell is meant by 'labels'

    adj_matrix = nx.adjacency_matrix(G, nodelist=sherd_pile) # SciPy adjacency matrix, ordered using sherd_pile
    print("SciPy adjacency matrix: {}".format(adj_matrix[0:20]))

    # num_sherds = number of sherds, i.e. groups of connected pixels
    # sherd_labels = sherd to which each pixel belongs (sherd 0, sherd 1, etc.), sorted in same order as sherd_pile
    num_sherds, sherd_labels = connected_components(adj_matrix, return_labels=True)

    print("Number of sherds: {}; Abbreviated labels: {}.".format(num_groups, group_labels[0:99]))

    sherds_and_pixel_cords = {}
    for i in range(0,num_sherds):
        indices = [j for j in range(len(sherd_labels)) if sherd_labels[j] == i]  # indices of all pixels labeled as sherd i
        sherds_and_pixel_cords[i] = [sherd_pile[index] for index in indices]

    print(sherds_and_pixel_cords)

    # get indices of all like labels; slice into sherd_pile; draw bounding box around each group of connected pixels.  Use polygon? 

"""
for char in scene_counts.keys():
    if scene_counts[char] > 0:
        midsummer.add_node(char, size = scene_counts[char])# For each co-appearance between two characters, add an edge
for char in appearance_counts.keys():
    for co_char in appearance_counts[char].keys():
        
        # Only add edge if the count is positive
        if appearance_counts[char][co_char] > 0:
            midsummer.add_edge(char, co_char), weight = appearance_counts[char][co_char])

G = nx.Graph()  # or DiGraph, MultiGraph, MultiDiGraph, etc.
G.add_node(1)
G.add_node('Hello')
K3 = nx.Graph([(0, 1), (1, 2), (2, 0)])
G.add_node(K3)
G.number_of_nodes()
"""

# get connected components

# draw bounding boxes around them

"""
row_dist_elements = row_dist.shape[0]*row_dist.shape[1]
col_dist_elements = col_dist.shape[0]*col_dist.shape[1]

#initialize adjacency matrices
row_adj = np.zeros( (row_dist_elements, row_dist_elements), dtype='int8' )
col_adj = np.zeros( (col_dist_elements, col_dist_elements), dtype='int8' )

print("col_adj shape is {}.".format(col_adj.shape))
print(col_adj)

if i == 0 or i == m:
    func edge_pixel
else:
    func interior_pixel

func edge_pixel
    if i == k:
        adjacency[i,k] = 0
    else:
        if distance[i,j] < eps:
            adjacency [i,k] = 1
        else:
            adjacency [i,k] = 0



print("col_dist has shape {} and looks like: {}".format(col_dist.shape, col_dist))

# construct graph

G = nx.from_numpy_matrix(col_dist)
G = nx.relabel_nodes(G, dict(zip(range(len(G.nodes())),string.ascii_uppercase)))    

G = nx.drawing.nx_agraph.to_agraph(G)

G.node_attr.update(color="red", style="filled")
G.edge_attr.update(color="blue", width="2.0")

G.draw('/tmp/out.png', format='png', prog='neato')

(col_np.sum(np.square(row_diff), axis=2)
print("Test: {}".format(np.sum(np.square(row_diff))))
col_dist = math.sqrt(np.sum(np.square(col_diff), axis=2))
"""

# pandas and scipy create a square distance matrix with distance between every pixel: https://stackoverflow.com/questions/29481485/creating-a-distance-matrix

"""
pixel_cords = []
xyz_cords = []
full = 255

for i in xrange(0,img_height):
    for j in xrange(0,img_width):
        if not image[i,j,0] == full and image[i,j,1] == full and image[i,j,2] == full:
            pixel_cords.append((i,j))
            xyz_cords.append( [ image[i,j,0], image[i,j,1], image[i,j,2] ] )
        
print('pixel_cords has length {} and xyz_cords has length {}.'.format(len(pixel_cords), len(xyz_cords)))
print('pixel_cords: {}'.format(pixel_cords))

df = pd.DataFrame(xyz_cords, columns=['xcord', 'ycord', 'zcord'], index=pixel_cords)
print(df)
dist_matrix = pd.DataFrame(distance_matrix(df.values, df.values), index=df.index, columns=df.index)
print(dist_matrix)

"""
