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

    K_P = 1.2
    K_D = 0.6
    K_I = 0.02

    X_D = 1000

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

            print drone_camera_image

            drone_camera_image_as_rgb = cv2.cvtColor(drone_camera_image.copy(), cv2.COLOR_BGR2RGB)

            # TODO: DO IN OWN CLASS
            # Find hat in camera view
            found_object, object_bounding_box, object_center = find_object(drone_camera_image.copy())
            if found_object:
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
                    K_P += 0.01
                elif very_last_pressed_key == 1:  # Key Down
                    K_P -= 0.01
                elif very_last_pressed_key == 2:  # Key Left
                    K_D -= 0.01
                elif very_last_pressed_key == 3:  # Key Right
                    K_D += 0.01

                elif very_last_pressed_key == 32:  # Space
                    drone.land()
                elif very_last_pressed_key == 13:  # Enter
                    e_int = 0
                    drone.takeoff()

                else:
                    print very_last_pressed_key

            alt.append(drone.altitude)
            ts_alt.append(current_millis() / 1000.0)

            if len(alt) > 2:
                alt.pop(0)
                ts_alt.pop(0)

                vz = float(alt[1] - alt[0]) / float(ts_alt[1] - ts_alt[0])
            else:
                vz = 0.0

            show_on_image(drone_camera_image_as_rgb, drone.ctrl_state, drone.phi, drone.psi, drone.theta, drone.altitude, drone.vx, drone.vy, drone.vz, drone.battery, drone.num_frames)

            show_text(drone_camera_image_as_rgb, 'Command: %s' % (very_last_pressed_key if very_last_pressed_key else '-',), (5, 135))
            show_text(drone_camera_image_as_rgb, 'Should hold altitude: %s' % should_hold_altitude, (5, 150))

            if object_center:
                # draw point in viewport
                # cv2.circle(drone_camera_image_as_rgb, object_center, 5.0, (255, 0, 0), 3, 0)

                object_center_delta_z = H / 2 - object_center[1]

                if abs(object_center_delta_z) > 25:
                    X_D = drone.altitude + object_center_delta_z * 5  # TODO: make factor 5 dependent on distance / size of entire

            elif (current_millis() - last_hat_time) > 10000:
                X_D = constants.DRONE_DEFAULT_ALTITUDE

            X_D = max(300, X_D)
            e_int += (X_D - drone.altitude) * (ts_alt[1] - ts_alt[0])
            e_int = max(e_int, 200)
            u = K_P * (X_D - drone.altitude) + K_D * (0 - vz) + K_I * e_int

            print drone.altitude, X_D, K_P, K_D, u

            if should_hold_altitude and key_up:
                apply_z_velocity(drone, u, vz)  # max(min(1.0, u / 1000.0), -1.0))

            # cv2.circle(drone_camera_image_as_rgb, (W / 2, H / 2), 25, (0, 255, 0), 2, )

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


def apply_z_velocity(drone, u, vz):
    drone.at(at_pcmd, True, 0, 0, min(max(u / 100.0, -1.0), 1.0), 0)


def find_object(image):
    found_hat = False
    hat_bounding_box = None
    hat_center = None

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
            hat_center = (x + (w / 2), (y + (h / 2)))

    return found_hat, hat_bounding_box, hat_center


def save_snapshot(image):
    cv2.imwrite('tmp/DroneView.jpg', image)


def current_millis():
    return int(round(time.time() * 1000))


if __name__ == '__main__':
    main()
