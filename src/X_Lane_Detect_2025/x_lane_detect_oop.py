import numpy as np
import cv2
import time

class LaneDetection:
    def __init__(self, width=640, height=480, offset=330):
        self.width = width
        self.height = height
        self.offset = offset

    def draw_rectangle(self, img, lpos, rpos, offset=0):
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
        cv2.rectangle(img, (self.width // 2 + 5, 15 + offset),
                           (self.width // 2 + 15, 25 + offset),
                           (0, 0, 255), 2)
        return img

    def make_coordinates(self, line_parameters):
        slope, y_intercept, x_intercept = line_parameters
        y2 = self.height * 2 // 5
        if slope < 0:
            y1 = int(y_intercept)
            x1 = int((y1 - y_intercept) / slope)
            x2 = int((y2 - y_intercept) / slope)
        elif slope > 0:
            y1 = int(slope * self.width + y_intercept)
            x1 = int((y1 - y_intercept) / slope)
            x2 = int((y2 - y_intercept) / slope)
        else:
            y1 = self.height
            x1 = int(x_intercept)
            x2 = int(x_intercept)
        return np.array([x1, y1, x2, y2])

    def average_slope_intercept(self, lines):
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
        left_fit_average = np.average(left_fit, axis=0) if left_fit else np.array([0, 0, 0])
        right_fit_average = np.average(right_fit, axis=0) if right_fit else np.array([0, 0, self.width])
        return np.array([left_fit_average, right_fit_average])

    def get_line(self, fit_average):
        left_line = self.make_coordinates(fit_average[0])
        right_line = self.make_coordinates(fit_average[1])
        return np.array([left_line, right_line])

    def canny(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 50, 150)
        return canny

    def region_of_interest(self, image):
        polygons = np.array([
            [(0, self.height), (0, 350), (170, 300), (250, 300), (140, self.height), 
             (560, self.height), (420, 300), (560, 300), (self.width, 350), (self.width, self.height)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def display_lines(self, image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
        return line_image

    def process_image(self, frame):
        lane_image = np.copy(frame)
        canny_image = self.canny(lane_image)
        cropped_image = self.region_of_interest(canny_image)
        lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)
        fit_average_lines = self.average_slope_intercept(lines)
        averaged_lines = self.get_line(fit_average_lines)
        line_image = self.display_lines(lane_image, averaged_lines)
        frame = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
        cv2.imshow("Region of Interest", cropped_image)
        cv2.imshow('Lane Detect', frame)
        cv2.waitKey(1)

        left_slope, left_y_intercept, left_x_intercept = fit_average_lines[0]
        right_slope, right_y_intercept, right_x_intercept = fit_average_lines[1]
        lpos = int(((self.offset + 20) - left_y_intercept) / left_slope) if left_slope != 0 else int(left_x_intercept)
        rpos = int(((self.offset + 20) - right_y_intercept) / right_slope) if right_slope != 0 else int(right_x_intercept)
        frame = self.draw_rectangle(frame, lpos, rpos, offset=self.offset)
        return (lpos, rpos), frame, [left_slope, right_slope]

class Steering:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.arrow_pic = cv2.imread('X_Lane_Detect/image/navin_steer.png', cv2.IMREAD_COLOR)

    def draw_steer(self, image, steer_angle):
        origin_height = self.arrow_pic.shape[0]
        origin_width = self.arrow_pic.shape[1]
        steer_wheel_center = origin_height * 0.74
        arrow_height = self.height // 2
        arrow_width = (arrow_height * 462) // 728 + 10

        matrix = cv2.getRotationMatrix2D((origin_width // 2, steer_wheel_center), steer_angle * 1.5, 0.7)
        arrow_pic_rotated = cv2.warpAffine(self.arrow_pic, matrix, (origin_width + 60, origin_height))
        arrow_pic_resized = cv2.resize(arrow_pic_rotated, (arrow_width, arrow_height), interpolation=cv2.INTER_AREA)

        gray_arrow = cv2.cvtColor(arrow_pic_resized, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

        arrow_roi = image[arrow_height: self.height, (self.width // 2 - arrow_width // 2): (self.width // 2 + arrow_width // 2)]
        arrow_roi = cv2.add(arrow_pic_resized, arrow_roi, mask=mask)
        res = cv2.add(arrow_roi, arrow_pic_resized)
        image[(self.height - arrow_height): self.height, (self.width // 2 - arrow_width // 2): (self.width // 2 + arrow_width // 2)] = res

        cv2.imshow('Steer Control', image)

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    time.sleep(3)

    lane_detector = LaneDetection()
    steering = Steering()

    while True:
        ret, image = cap.read()
        if not ret:
            break
        pos, frame, slope = lane_detector.process_image(image)
        steer_angle = -(pos[1] + pos[0] - lane_detector.width) / 5
        steering.draw_steer(frame, steer_angle)

        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
