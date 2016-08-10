#!/usr/bin/env python
import logging
import time

import cv2
import libardrone.libardrone as libardrone
import numpy as np
from libardrone.libardrone import at_pcmd


def main():
    # Constants
    W, H = 640, 360

    K_P = 1.2
    K_D = 0.6
    K_I = 0.02

    DEFAULT_SPEED = 0.1

    #
    X_D = 1000

    running = True
    should_hold_altitude = False

    last_pressed_keys = []

    last_hat_time = 0

    alt = [0]
    ts_alt = [current_millis() / 1000.0]

    e_int = 0

    # Init & Setup drone
    drone = libardrone.ARDrone2()
    drone.reset()
    drone.set_speed(DEFAULT_SPEED)

    # bounding_rect_start, bounding_rect_end = ((160, 320), (160, 320))

    try:
        while running:
            drone_camera_image = drone.get_image()

            drone_camera_image_as_rgb = cv2.cvtColor(drone_camera_image.copy(), cv2.COLOR_BGR2RGB)

            # Find hat in camera view
            found_hat, hat_bounding_box = find_hat(drone_camera_image.copy())

            center_hat = None

            if found_hat:
                x, y, w, h = hat_bounding_box
                center_hat = (x + (w / 2), (y + h / 2))
                last_hat_time = current_millis()

            # Key Press Events
            pressed_key = cv2.waitKey(15) & 0xFF

            last_pressed_keys.append(pressed_key)

            if len(last_pressed_keys) > 8:
                last_pressed_keys.pop(0)

            key_up = all(last_pressed_key == 255 for last_pressed_key in last_pressed_keys)

            very_last_pressed_key = None

            if not key_up:
                for last_pressed_key in last_pressed_keys:
                    if last_pressed_key != 255:
                        very_last_pressed_key = last_pressed_key

                drone.set_speed(0.15)

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

                elif very_last_pressed_key == 0:  # Key Up
                    # drone.move_up()
                    K_P += 0.01
                elif very_last_pressed_key == 1:  # Key Down
                    # drone.move_down()
                    K_P -= 0.01
                elif very_last_pressed_key == 2:  # Key Left
                    # drone.turn_left()
                    K_D -= 0.01
                elif very_last_pressed_key == 3:  # Key Right
                    # drone.turn_right()
                    K_D += 0.01

                elif very_last_pressed_key == 32:  # Space
                    drone.land()
                elif very_last_pressed_key == 13:  # Enter
                    e_int = 0
                    drone.takeoff()

                elif very_last_pressed_key == ord('i'):
                    save_snapshot(drone_camera_image_as_rgb)

                elif very_last_pressed_key == ord('v'):
                    should_hold_altitude = True

                elif very_last_pressed_key == ord('n'):
                    should_hold_altitude = False

                elif very_last_pressed_key == ord('r'):
                    drone.reset()

                elif very_last_pressed_key == ord('h'):
                    drone.hover()

                else:
                    print very_last_pressed_key

            nav_data = drone.get_navdata()
            nav_data = nav_data[0]

            ctrl_state = nav_data['ctrl_state']
            phi = nav_data['phi']
            psi = nav_data['psi']
            theta = nav_data['theta']
            altitude = nav_data['altitude']
            vx = nav_data['vx']
            vy = nav_data['vy']
            vz = nav_data['vz']
            battery = nav_data['battery']
            num_frames = nav_data['num_frames']

            alt.append(altitude)
            ts_alt.append(current_millis() / 1000.0)

            if len(alt) > 2:
                alt.pop(0)
                ts_alt.pop(0)

                vz = float(alt[1] - alt[0]) / float(ts_alt[1] - ts_alt[0])
                # print alt, ts_alt, ts_alt[1] - ts_alt[0], vz

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_size = 0.5

            cv2.putText(drone_camera_image_as_rgb, 'State: %s' % ctrl_state, (5, 15), font, font_size, (255, 255, 255))

            cv2.putText(drone_camera_image_as_rgb, 'Phi (+Roll Right/- Roll Left): %.0f' % phi, (5, 30), font, font_size, (255, 255, 255))
            cv2.putText(drone_camera_image_as_rgb, 'Psi (+Turn Right/- Turn Left): %.0f' % psi, (5, 45), font, font_size, (255, 255, 255))
            cv2.putText(drone_camera_image_as_rgb, 'Theta (+Up/-Down): %.0f' % theta, (5, 60), font, font_size, (255, 255, 255))

            cv2.putText(drone_camera_image_as_rgb, 'Altitude: %.0f' % altitude, (5, 75), font, font_size, (255, 255, 255))
            cv2.putText(drone_camera_image_as_rgb, 'vx/vy/vz: %.3f/%.3f/%.3f' % (vx, vy, vz), (5, 90), font, font_size, (255, 255, 255))

            cv2.putText(drone_camera_image_as_rgb, 'Battery: %.0f%%' % battery, (5, 105), font, font_size, (255, 255, 255) if battery > 20 else (0, 0, 255))

            cv2.putText(drone_camera_image_as_rgb, '# Frames (?): %.0f' % num_frames, (5, 120), font, font_size, (255, 255, 255))
            cv2.putText(drone_camera_image_as_rgb, 'Command: %s' % (very_last_pressed_key if very_last_pressed_key else '-',), (5, 135), font, font_size, (255, 255, 255))
            cv2.putText(drone_camera_image_as_rgb, 'Should hold altitude: %s' % should_hold_altitude, (5, 150), font, font_size, (255, 255, 255))

            if center_hat:
                # draw point in viewport
                cv2.rectangle(drone_camera_image_as_rgb, center_hat, center_hat, (255, 0, 0), 3, 0)
                delta_z = H / 2 - center_hat[1]
                if abs(delta_z) > 25:
                    X_D = altitude + delta_z * 5
            elif (current_millis() - last_hat_time) > 10000:
                X_D = 300

            # cv2.rectangle(image_real_shit, (x, y), (x + w, y + h), (200, 0, 0), 2)

            X_D = max(300, X_D)
            e_int += (X_D - altitude) * (ts_alt[1] - ts_alt[0])
            e_int = max(e_int, 200)
            u = K_P * (X_D - altitude) + K_D * (0 - vz) + K_I * e_int

            print altitude, X_D, K_P, K_D, u

            if should_hold_altitude and key_up:
                apply_z_velocity(drone, u, vz)  # max(min(1.0, u / 1000.0), -1.0))

            cv2.rectangle(drone_camera_image_as_rgb, (W / 2 - 25, H / 2 - 25), (W / 2 + 25, H / 2 + 25), (0, 255, 0), 2, )
            # cv2.rectangle(image_real_shit, bounding_rect_start, bounding_rect_end, (0, 255, 0), 2, )

            cv2.imshow('Live Drone Cam (Jeronimo)', drone_camera_image_as_rgb)

    except Exception, e:
        print 'Exception happened in main while loop:'
        logging.error(e, exc_info=True)

    cv2.destroyAllWindows()

    print("Shutting down...")
    drone.reset()
    drone.halt()
    print("Shut down.")


def apply_z_velocity(drone, u, vz):
    drone.at(at_pcmd, True, 0, 0, min(max(u / 100.0, -1.0), 1.0), 0)


def find_hat(image):
    found_hat = False
    hat_bounding_box = None

    # Convert to HSV in order to filter by color
    image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # Filter by color red
    lower_red_1 = np.array([0, 50, 50])
    upper_red_1 = np.array([5, 255, 255])

    image = cv2.inRange(image, lower_red_1, upper_red_1)

    # Put on median blur to reduce noise
    image = cv2.medianBlur(image, 15)

    # Find contours and decide if hat is one of them
    contours, hierarchy = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # 1) Find biggest contour
    biggest_contour = None
    biggest_contour_size = 0

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        if w * h > biggest_contour_size:
            biggest_contour = contour

    # 2) Check if biggest contour is hat
    if biggest_contour is not None:
        x, y, w, h = cv2.boundingRect(biggest_contour)

        if w > 40 and h > 40:
            found_hat = True
            hat_bounding_box = cv2.boundingRect(biggest_contour)

    return found_hat, hat_bounding_box


def save_snapshot(image):
    cv2.imwrite('tmp/DroneView.jpg', image)


def current_millis():
    return int(round(time.time() * 1000))


if __name__ == '__main__':
    main()
