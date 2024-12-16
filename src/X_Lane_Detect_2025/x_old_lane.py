import numpy as np
import cv2
import time

Width = 640
Height = 480
Offset = 330

# Draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) // 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)   
    cv2.rectangle(img, ((rpos + lpos) // 2 - 5, 15 + offset),
                       ((rpos + lpos) // 2 + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (Width // 2 + 5, 15 + offset),
                       (Width // 2 + 15, 25 + offset),
                       (0, 0, 255), 2)
    return img

# Create coordinates for line
def make_coordinates(image, line_parameters):
    slope, y_intercept, x_intercept = line_parameters
    y2 = Height * 2 // 5
    if slope < 0:
        y1 = int(y_intercept)
        x1 = int((y1 - y_intercept) / slope)
        x2 = int((y2 - y_intercept) / slope)
    elif slope > 0:
        y1 = int(slope * Width + y_intercept)
        x1 = int((y1 - y_intercept) / slope)
        x2 = int((y2 - y_intercept) / slope)
    else:
        y1 = Height
        x1 = int(x_intercept)
        x2 = int(x_intercept)
    return np.array([x1, y1, x2, y2])

# Calculate average slope and intercept
def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_intercept = parameters[1]
        x_intercept = -y_intercept / slope
        if slope < -0.31 and slope > -0.75:
            left_fit.append((slope, y_intercept, x_intercept))
        elif slope > 0.31 and slope < 0.75:
            right_fit.append((slope, y_intercept, x_intercept))
    if not left_fit:
        left_fit_average = np.array([0, 0, 0])
    else:
        left_fit_average = np.average(left_fit, axis=0)
    if not right_fit:
        right_fit_average = np.array([0, 0, Width])
    else:
        right_fit_average = np.average(right_fit, axis=0)

    return np.array([left_fit_average, right_fit_average])

# Get line coordinates
def get_line(fit_average):
    left_line = make_coordinates(image, fit_average[0])
    right_line = make_coordinates(image, fit_average[1])
    return np.array([left_line, right_line])

# Canny edge detection
def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

# Region of interest
def region_of_interest(image):
    polygons = np.array([
        [(0, Height), (0, 350), (170, 300), (250, 300), (140, Height), (560, Height), (420, 300), (560, 300), (Width, 350), (Width, Height)]
    ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

# Display lines on image
def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # Convert coordinates to integers
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_image

# Process image to find lanes and draw results
def process_image(frame):
    global Offset

    lane_image = np.copy(frame)
    canny_image = canny(lane_image)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    fit_average_lines = average_slope_intercept(lane_image, lines)
    averaged_lines = get_line(fit_average_lines)
    line_image = display_lines(lane_image, averaged_lines)
    frame = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
    cv2.imshow("Region of Interest", cropped_image)
    cv2.imshow('Lane Detect', frame)
    cv2.waitKey(1)

    left_slope, left_y_intercept, left_x_intercept = fit_average_lines[0]
    right_slope, right_y_intercept, right_x_intercept = fit_average_lines[1]
    if left_slope != 0:
        lpos = int(((Offset + 20) - left_y_intercept) / left_slope)
    else:
        lpos = int(left_x_intercept)
    if right_slope != 0:
        rpos = int(((Offset + 20) - right_y_intercept) / right_slope)
    else:
        rpos = int(right_x_intercept)
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    return (lpos, rpos), frame, [left_slope, right_slope]

# Draw steering arrow
def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    arrow_pic = cv2.imread('image/navin_steer.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height // 2
    arrow_Width = (arrow_Height * 462) // 728 + 10

    matrix = cv2.getRotationMatrix2D((origin_Width // 2, steer_wheel_center), steer_angle * 1.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width + 60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, (arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width // 2 - arrow_Width // 2): (Width // 2 + arrow_Width // 2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width // 2 - arrow_Width // 2): (Width // 2 + arrow_Width // 2)] = res

    cv2.imshow('Steer Control', image)

if __name__ == '__main__':
    cap = cv2.VideoCapture('video/navin_testing_1.mkv')
    time.sleep(3)

    while True:
        ret, image = cap.read()
        if not ret:
            break
        pos, frame, slope = process_image(image)
        steer_angle = -(pos[1] + pos[0] - Width) / 5
        draw_steer(frame, steer_angle)

        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
