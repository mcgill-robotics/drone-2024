import cv2
import numpy as np


def read_color_on_shape(letter_crop):
    # process object
    hsv_img = cv2.cvtColor(letter_crop, cv2.COLOR_BGR2HSV)

    colors = {
        'red': 0.,
        'green': 0.,
        'blue': 0.,
        'white': 0.,
        'black': 0.,
        'purple': 0.,
        'brown': 0.,
        'orange': 0.
    }

    # red boundaries
    red_lower1 = np.array([0, 50, 50])
    red_upper1 = np.array([5, 255, 255])
    red_lower2 = np.array([174, 50, 50])
    red_upper2 = np.array([179, 255, 255])
    red_lower_mask = cv2.inRange(hsv_img, red_lower1, red_upper1)
    red_upper_mask = cv2.inRange(hsv_img, red_lower2, red_upper2)
    red_mask = red_lower_mask + red_upper_mask

    # green boundaries
    green_lower = np.array([51, 50, 50], np.uint8)
    green_upper = np.array([71, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsv_img, green_lower, green_upper)

    # blue boundaries
    blue_lower = np.array([63, 50, 50], np.uint8)
    blue_upper = np.array([90, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)

    # white boundaries
    white_lower = np.array([0, 0, 127], np.uint8)
    white_upper = np.array([179, 20, 255], np.uint8)
    white_mask = cv2.inRange(hsv_img, white_lower, white_upper)

    # black boundaries
    black_lower = np.array([0, 0, 0], np.uint8)
    black_upper = np.array([179, 127, 127], np.uint8)
    black_mask = cv2.inRange(hsv_img, black_lower, black_upper)

    # purple boundaries
    purple_lower = np.array([139, 50, 50], np.uint8)
    purple_upper = np.array([153, 255, 255], np.uint8)
    purple_mask = cv2.inRange(hsv_img, purple_lower, purple_upper)

    # brown boundaries
    brown_lower = np.array([15, 105, 105], np.uint8)
    brown_upper = np.array([40, 153, 153], np.uint8)
    brown_mask = cv2.inRange(hsv_img, brown_lower, brown_upper)
    # orange boundaries
    orange_lower = np.array([15, 160, 160], np.uint8)
    orange_upper = np.array([40, 255, 255], np.uint8)
    orange_mask = cv2.inRange(hsv_img, orange_lower, orange_upper)
    cv2.imshow("image", orange_mask)
    cv2.waitKey(0)

    colors['red'] = np.sum(red_mask)
    colors['green'] = np.sum(red_mask)
    colors['blue'] = np.sum(red_mask)
    colors['white'] = np.sum(red_mask)
    colors['black'] = np.sum(red_mask)
    colors['purple'] = np.sum(red_mask)
    colors['brown'] = np.sum(red_mask)
    # For red     # creating contour to track red color
    # contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE,
    #                                        cv2.CHAIN_APPROX_SIMPLE)
    # for contour in contours:
    #     area = cv2.contourArea(contour)
    #     colors['red'] += area
    #
    # # Creating contour to track green color
    # contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE,
    #                                        cv2.CHAIN_APPROX_SIMPLE)
    # for contour in (contours):
    #     area = cv2.contourArea(contour)
    #     colors['green'] += area
    #
    # # Creating contour to track blue color
    # contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE,
    #                                        cv2.CHAIN_APPROX_SIMPLE)
    # for contour in (contours):
    #     area = cv2.contourArea(contour)
    #     colors['blue'] += area
    #
    # # creating contour to track white color
    # contours, hierarchy = cv2.findContours(white_mask, cv2.RETR_TREE,
    #                                        cv2.CHAIN_APPROX_SIMPLE)
    # for contour in (contours):
    #     area = cv2.contourArea(contour)
    #     colors['white'] += area
    #
    # # Creating contour to track black color
    # contours, hierarchy = cv2.findContours(black_mask, cv2.RETR_TREE,
    #                                        cv2.CHAIN_APPROX_SIMPLE)
    # for contour in (contours):
    #     area = cv2.contourArea(contour)
    #     colors['black'] += area
    #
    # # Creating contour to track purple color
    # contours, hierarchy = cv2.findContours(purple_mask, cv2.RETR_TREE,
    #                                        cv2.CHAIN_APPROX_SIMPLE)
    # for contour in (contours):
    #     area = cv2.contourArea(contour)
    #     colors['purple'] += area
    #
    # # creating contour to track brown color
    # contours, hierarchy = cv2.findContours(brown_mask, cv2.RETR_TREE,
    #                                        cv2.CHAIN_APPROX_SIMPLE)
    # for contour in (contours):
    #     area = cv2.contourArea(contour)
    #     colors['brown'] += area
    #
    # cv2.drawContours(brown_mask, contours, -1, (0, 255, 0), 3)
    # cv2.waitKey(0)
    # Creating contour to track orange color
    contours, hierarchy = cv2.findContours(orange_mask, cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for contour in (contours):
        area = cv2.contourArea(contour)
        colors['orange'] += area

    shape_color = None
    shape_color_area = -1
    text_color = None
    text_color_area = -1

    for color, tot_area in (colors.items()):
        if (shape_color_area < tot_area):
            text_color_area = shape_color_area
            text_color = shape_color

            shape_color_area = tot_area
            shape_color = color

    print(colors)

    return shape_color, text_color


def main():
    img = cv2.imread(
        "/home/unhappysquid/drone_2024/onboard_ws/src/computer_vision/computer_vision/brown_circle.png",
        cv2.IMREAD_COLOR)
    print(read_color_on_shape(img))
    cv2.imshow("image", img)

    cv2.waitKey(0)


if __name__ == "__main__":
    main()
