# Colors for marking graphs
RED = (50, 50, 180)
YELLOW = (0, 175, 205)
GREEN = (50, 180, 40)
GREY = (50, 50, 50)

def mark_path(path, image):
    for i in range(len(path)):
        mark_cell(image, path[i][0], path[i][1], YELLOW, 4)

def mark_cell(image, blobX, blobY, color, width):
    for i in range(-width, width+1):
        for j in range(-width, width+1):
            x = int(blobX) + i
            y = int(blobY) + j
            if (x > 0 and x < len(image[0]) and y > 0 and y < len(image)):
                image[y][x] = color
