#!/usr/bin/env python
import logging

import libardrone.libardrone as libardrone
import numpy as np
import cv2

W, H = 360, 640

K_P = 0.1
X_D = 1000


def main():
    drone = libardrone.ARDrone2()
    drone.reset()
    drone.set_speed(0.1)

    try:
        running = True

        last_pressed_keys = []

        bounding_rect_start, bounding_rect_end = ((160, 320), (160, 320))

        while running:
            image = drone.get_image()
            image_real_shit = image.copy()
            image_real_shit = cv2.cvtColor(image_real_shit, cv2.COLOR_BGR2RGB)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)  # COLOR_BGR2RGB

            lower_red_1 = np.array([0, 50, 50])
            upper_red_1 = np.array([5, 255, 255])

            # lower_red_2 = np.array([160, 50, 50])
            # upper_red_2 = np.array([179, 255, 255])

            image = cv2.inRange(image, lower_red_1, upper_red_1)

            image = cv2.medianBlur(image, 15)
            # image = cv2.inRange(image, lower_red_2, upper_red_2)
            #
            # image = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0)

            # image = cv2.GaussianBlur(image, (9, 9), 2)

            # lower_blue = np.array([50, 5, 5])
            # upper_blue = np.array([190, 255, 255])
            #
            # image = cv2.inRange(image, lower_blue, upper_blue)

            # image = cv2.GaussianBlur(image, (9, 9), 2)
            # cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

            # Bitwise-AND mask and original image
            # image = cv2.bitw(image, image, mask=mask1)
            # image = cv2.bitwise_and(image, image, mask=mask1)

            # img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

            # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            contours, hierarchy = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            biggest_contour = None
            biggest_contour_size = 0

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)

                if w * h > biggest_contour_size:
                    biggest_contour = contour

            if biggest_contour is not None:
                x, y, w, h = cv2.boundingRect(biggest_contour)
                cv2.rectangle(image_real_shit, (x, y), (x + w, y + h), (200, 0, 0), 2)

            # img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #
            # # lower mask (0-10)
            # lower_red = np.array([0, 50, 50])
            # upper_red = np.array([10, 255, 255])
            # mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
            #
            # # upper mask (170-180)
            # lower_red = np.array([170, 50, 50])
            # upper_red = np.array([180, 255, 255])
            # mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
            #
            # # join my masks
            # mask = mask0 + mask1
            #
            # # set my output img to zero everywhere except my mask
            # # image = image.copy()
            # image[np.where(mask == 0)] = 0
            #
            # # or your HSV image, which I *believe* is what you want
            # output_hsv = img_hsv.copy()
            # output_hsv[np.where(mask == 0)] = 0

            pressed_key = cv2.waitKey(15) & 0xFF

            last_pressed_keys.append(pressed_key)

            if len(last_pressed_keys) > 8:
                last_pressed_keys.pop(0)

            key_up = all(last_pressed_key == 255 for last_pressed_key in last_pressed_keys)

            # print last_pressed_keys, key_up

            very_last_pressed_key = None

            if not key_up:
                for last_pressed_key in last_pressed_keys:
                    if last_pressed_key != 255:
                        very_last_pressed_key = last_pressed_key

                # print very_last_pressed_key

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
                    drone.move_up()
                elif very_last_pressed_key == 1:  # Key Down
                    drone.move_down()
                elif very_last_pressed_key == 2:  # Key Left
                    drone.turn_left()
                elif very_last_pressed_key == 3:  # Key Right
                    drone.turn_right()
                elif very_last_pressed_key == 32:  # Space
                    drone.land()
                elif very_last_pressed_key == 13:  # Enter
                    drone.takeoff()

                elif very_last_pressed_key == ord('i'):
                    bounding_rect_start, bounding_rect_end = get_prediction(image)
                elif very_last_pressed_key == ord('r'):
                    drone.reset()
                else:
                    print very_last_pressed_key
            else:
                applyZVelocity(drone, 0)

            try:
                nav_data = drone.get_navdata()

                nav_data = nav_data[0]

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_size = 0.5

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

                cv2.putText(image_real_shit, 'State: %s' % ctrl_state, (5, 15), font, font_size, (255, 255, 255))

                cv2.putText(image_real_shit, 'Phi (+Roll Right/- Roll Left): %.0f' % phi, (5, 30), font, font_size, (255, 255, 255))
                cv2.putText(image_real_shit, 'Psi (+Turn Right/- Turn Left): %.0f' % psi, (5, 45), font, font_size, (255, 255, 255))
                cv2.putText(image_real_shit, 'Theta (+Up/-Down): %.0f' % theta, (5, 60), font, font_size, (255, 255, 255))

                cv2.putText(image_real_shit, 'Altitude: %.0f' % altitude, (5, 75), font, font_size, (255, 255, 255))
                cv2.putText(image_real_shit, 'vx/vy/vz: %.3f/%.3f/%.3f' % (vx, vy, vz), (5, 90), font, font_size, (255, 255, 255))

                cv2.putText(image_real_shit, 'Battery: %.0f%%' % battery, (5, 105), font, font_size, (255, 255, 255))

                cv2.putText(image_real_shit, '# Frames (?): %.0f' % num_frames, (5, 120), font, font_size, (255, 255, 255))
                cv2.putText(image_real_shit, 'Command: %s' % (very_last_pressed_key if very_last_pressed_key else '-',), (5, 135), font, font_size, (255, 255, 255))

                # = 0.1 * (1000 - ...)
                u = K_P * (X_D - altitude)

                print nav_data['altitude'], u

            except Exception, e:
                print 'Exception happened while trying to access nav data:'
                logging.error(e, exc_info=True)

            cv2.rectangle(image_real_shit, (H / 2 - 25, W / 2 - 25), (H / 2 + 25, W / 2 + 25), (0, 255, 0), 2, )
            cv2.rectangle(image_real_shit, bounding_rect_start, bounding_rect_end, (0, 255, 0), 2, )

            cv2.imshow('Live Drone Cam (Jeronimo)', image_real_shit)

    except Exception, e:
        print 'Exception happened in main while loop:'
        logging.error(e, exc_info=True)

    cv2.destroyAllWindows()

    print("Shutting down...")
    drone.reset()
    drone.halt()
    print("Shut down.")


network_mutex = True


def get_prediction(image):
    if not network_mutex:
        print network_mutex

    cv2.imwrite('tmp/DroneView.jpg', image)

    # image_file = open('tmp/DroneView.jpg', 'rb')

    # HOST = 'localhost'
    # PORT = '5000'
    # JOB_ID = '20160809-164902-44b3'

    # print "start request..."
    # r = requests.post('http://%s:%s/models/images/classification/classify_one.json' % (
    #     HOST,
    #     PORT,
    # ), data={
    #     'job_id': JOB_ID,
    # }, files={
    #     'image_file': image_file,
    # })
    #
    # print r.text  # text['predictions']

    return (H / 2 - 24, W / 2 - 24), (H / 2 + 26, W / 2 + 26)


def applyZVelocity(drone, v_dest):
    if abs(v_dest) < 0.1:
        drone.hover()
        return

    drone.set_speed(abs(v_dest))
    if v_dest >= 0:
        drone.moveDown()
    else:
        drone.moveUpd()


if __name__ == '__main__':
    main()
