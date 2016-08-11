#!/usr/bin/env python
import logging
import time

import cv2
import numpy as np
from libardrone.libardrone import at_pcmd, ARDrone2

import constants
from drone_wrapper import ARDroneWrapper


def main():
    # Constants
    W, H = 640, 360

    K_P = 0.6
    K_D = 0.6
    K_I = 0.02

    X_D = 1000.0

    speex_x_turn_factor = 0.2

    running = True
    should_hold_altitude = False

    last_pressed_keys = []

    last_hat_time = 0

    alt = [0]
    ts_alt = [current_millis() / 1000.0]

    e_int = 0

    # Init & Setup drone
    drone = ARDroneWrapper()
    drone.setup()

    try:
        while running:
            drone_camera_image = drone.get_image()

            drone_camera_image_as_rgb = cv2.cvtColor(drone_camera_image.copy(), cv2.COLOR_BGR2RGB)

            # TODO: DO IN OWN CLASS
            # Find hat in camera view
            found_object, object_bounding_box, object_center = find_object(drone_camera_image.copy())
            if found_object:
                last_hat_time = current_millis()

            # Key Press Events
            pressed_key = cv2.waitKey(2) & 0xFF

            last_pressed_keys.append(pressed_key)

            if len(last_pressed_keys) > 8:
                last_pressed_keys.pop(0)

            key_up = all(last_pressed_key == 255 for last_pressed_key in last_pressed_keys)

            very_last_pressed_key = None

            if not key_up:
                for last_pressed_key in last_pressed_keys:
                    if last_pressed_key != 255:
                        very_last_pressed_key = last_pressed_key

                drone.set_speed(0.2)

                if very_last_pressed_key == ord('q'):  # Q
                    running = False
                elif very_last_pressed_key == ord('w'):  # W
                    drone.move_forward()
                elif very_last_pressed_key == ord('s'):  # S
                    drone.move_backward()
                elif very_last_pressed_key == ord('a'):  # A
                    drone.move_left()
                elif very_last_pressed_key == ord('d'):  # D
                    drone.move_right()

                elif very_last_pressed_key == ord('v'):  # V
                    should_hold_altitude = True
                elif very_last_pressed_key == ord('n'):  # N
                    should_hold_altitude = False

                elif very_last_pressed_key == ord('r'):  # R
                    drone.reset()

                elif very_last_pressed_key == ord('h'):  # H
                    drone.hover()

                elif very_last_pressed_key == ord('i'):  # I
                    save_snapshot(drone_camera_image_as_rgb)

                elif very_last_pressed_key == 0:  # Key Up
                    drone.move_up()
                elif very_last_pressed_key == 1:  # Key Down
                    drone.move_down()
                elif very_last_pressed_key == 2:  # Key Left
                    drone.turn_left()
                elif very_last_pressed_key == 3:  # Key Right
                    drone.turn_right()

                elif very_last_pressed_key == 32:  # Space
                    drone.land()
                elif very_last_pressed_key == 13 or very_last_pressed_key == 10:  # Enter
                    e_int = 0
                    drone.takeoff()

                else:
                    print very_last_pressed_key

            # Update Altitude & Calc vz
            alt.append(drone.altitude)
            ts_alt.append(current_millis() / 1000.0)

            if len(alt) > 2:
                alt.pop(0)
                ts_alt.pop(0)

                vz = float(alt[1] - alt[0]) / float(ts_alt[1] - ts_alt[0])
            else:
                vz = 0.0

            speed_x = 0.0
            speed_y = 0.0
            speed_z = 0.0

            if object_center:
                center_x, center_y = object_center

                object_center_delta_width = W / 2 - center_x
                object_center_delta_width_rel = float(object_center_delta_width) / float(W / 2)

                object_center_delta_height = H / 2 - center_y
                object_center_delta_height_rel = float(object_center_delta_height) / float(H / 2)

                if abs(object_center_delta_height) > 25:
                    X_D = max(300, min(2000, drone.altitude + object_center_delta_height * 5))  # TODO: make factor 5 dependent on distance / size of entire
                    # print 'do something different:', object_center_delta_z

                if abs(object_center_delta_width_rel) > 0.2:
                    speed_x = - object_center_delta_width_rel  # right , left

                if abs(object_center_delta_height_rel) > 0.2:
                    speed_z = object_center_delta_height_rel  # up , down

                x, y, w, h = object_bounding_box

                # if float(w) / float(W) > 0.4:
                #     speed_y = 0.01  # back, forth
                # elif float(w) / float(W) < 0.275:
                #     speed_y = -0.01  # back, forth

                # print w, W, float(w) / float(W), speed_y

            elif (current_millis() - last_hat_time) > 7000:
                X_D = constants.DRONE_DEFAULT_ALTITUDE

            e_int += (X_D - drone.altitude) * (ts_alt[1] - ts_alt[0])
            e_int = max(e_int, 200)

            u = K_P * (X_D - drone.altitude) + K_D * (0 - vz) + K_I * e_int

            # print drone.altitude, X_D, u

            if should_hold_altitude and key_up:
                # print '>', X_D, drone.altitude, delta, speed
                # if abs(X_D - drone.altitude) > 50:
                #     delta = X_D - drone.altitude
                #     speed_z = float(delta) / float(X_D)

                speed_x *= 0.5  # 0.5
                speed_y *= 0.1  # 0.1
                speed_z *= 0.3  # 0.3

                print speed_x, speed_y, speed_z

                if any(abs(speed) > 0.1 for speed in (speed_x, speed_y, speed_z)):
                    drone.at(at_pcmd, True, speed_x * 0.1, speed_y, speed_z, speed_x)
                else:
                    drone.hover()

                    # apply_z_velocity(drone, u)  # max(min(1.0, u / 1000.0), -1.0))

            show_on_image(drone_camera_image_as_rgb, drone.ctrl_state, drone.phi, drone.psi, drone.theta, drone.altitude, drone.vx, drone.vy, drone.vz, drone.battery, drone.num_frames)

            show_text(drone_camera_image_as_rgb, 'Command: %s' % (very_last_pressed_key if very_last_pressed_key else '-',), (5, 135))
            show_text(drone_camera_image_as_rgb, 'Should hold altitude: %s' % should_hold_altitude, (5, 150))

            cv2.circle(drone_camera_image_as_rgb, (W / 2, H / 2), int(H * 0.2), (0, 255, 0), 2, )

            if object_center:
                # draw point in viewport
                cv2.circle(drone_camera_image_as_rgb, object_center, 5, (255, 0, 0), 3, 0)

            cv2.imshow('Live Drone Cam (Jeronimo)', drone_camera_image_as_rgb)

    except Exception, e:
        print 'Exception happened in main while loop:'
        logging.error(e, exc_info=True)

    cv2.destroyAllWindows()

    print("Shutting down...")
    drone.reset()
    drone.halt()
    print("Shut down.")


def show_on_image(image, ctrl_state, phi, psi, theta, altitude, vx, vy, vz, battery, num_frames):
    show_text(image, 'State: %s' % ctrl_state, (5, 15))

    cv2.putText(image, 'Phi (+Roll Right/- Roll Left): %.0f' % phi, (5, 30), constants.FONT, constants.FONT_SIZE, (255, 255, 255))
    cv2.putText(image, 'Psi (+Turn Right/- Turn Left): %.0f' % psi, (5, 45), constants.FONT, constants.FONT_SIZE, (255, 255, 255))
    cv2.putText(image, 'Theta (+Up/-Down): %.0f' % theta, (5, 60), constants.FONT, constants.FONT_SIZE, (255, 255, 255))

    cv2.putText(image, 'Altitude: %.0f' % altitude, (5, 75), constants.FONT, constants.FONT_SIZE, (255, 255, 255))
    cv2.putText(image, 'vx/vy/vz: %.3f/%.3f/%.3f' % (vx, vy, vz), (5, 90), constants.FONT, constants.FONT_SIZE, (255, 255, 255))

    cv2.putText(image, 'Battery: %.0f%%' % battery, (5, 105), constants.FONT, constants.FONT_SIZE, (255, 255, 255) if battery > 20 else (0, 0, 255))

    cv2.putText(image, '# Frames (?): %.0f' % num_frames, (5, 120), constants.FONT, constants.FONT_SIZE, (255, 255, 255))


def show_text(image, text, pt, font=constants.FONT, font_size=constants.FONT_SIZE, color=(255, 255, 255)):
    cv2.putText(image, text, pt, font, font_size, color)


def apply_z_velocity(drone, u):
    if abs(u) < 30:
        drone.hover()
    else:
        drone.at(at_pcmd, True, 0, 0, min(max(u / 100.0, -1.0), 1.0), 0)


def find_object(image):
    found_hat = False
    hat_bounding_box = None
    hat_center = None

    # Convert to HSV in order to filter by color
    image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    image = cv2.medianBlur(image, 3)

    # Filter by color red
    # lower_red = np.array([0, 50, 50])
    # upper_red = np.array([5, 255, 255])
    # image = cv2.inRange(image, lower_red, upper_red)

    lower_yellow = np.array([22, 120, 50])
    upper_yellow = np.array([27, 255, 255])
    image = cv2.inRange(image, lower_yellow, upper_yellow)

    # useley color
    # lower_blue = np.array([95, 50, 50])
    # upper_blue = np.array([100, 255, 255])
    # image = cv2.inRange(image, lower_blue, upper_blue)

    # Academy of Consult Journal
    # lower_bound = np.array([165, 50, 50])
    # upper_bound = np.array([170, 255, 255])
    # image = cv2.inRange(image, lower_bound, upper_bound)

    # Put on median blur to reduce noise
    # image = cv2.GaussianBlur(image, (15, 15), 2)
    image = cv2.medianBlur(image, 11)

    # Find contours and decide if hat is one of them
    contours, hierarchy = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # 1) Find biggest contour
    biggest_contour = None
    biggest_contour_size = 0

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        if w * h > biggest_contour_size:
            biggest_contour = contour
            biggest_contour_size = w * h

    # 2) Check if biggest contour is hat
    if biggest_contour is not None:
        x, y, w, h = cv2.boundingRect(biggest_contour)

        if w * h > 100:
            found_hat = True
            hat_bounding_box = cv2.boundingRect(biggest_contour)
            hat_center = (x + (w / 2), (y + (h / 2)))

    cv2.imshow("masked", image)

    return found_hat, hat_bounding_box, hat_center


def save_snapshot(image):
    cv2.imwrite('tmp/DroneView.jpg', image)


def current_millis():
    return int(round(time.time() * 1000))


def minmax(n, minn, maxn):
    return max(min(maxn, n), minn)


if __name__ == '__main__':
    main()
