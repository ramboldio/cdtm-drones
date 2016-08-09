#!/usr/bin/env python
import logging
from time import sleep

import libardrone.libardrone as libardrone

import cv2


def main():
    drone = libardrone.ARDrone2()
    drone.reset()
    drone.set_speed(0.1)

    try:
        running = True

        while running:
            image = drone.get_image()

            # print image

            image = cv2.cvtColor(image, cv2.IMREAD_COLOR)  # COLOR_BGR2RGB

            try:
                nav_data = drone.get_navdata()

                nav_data = nav_data[0]

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_size = 0.5

                cv2.putText(image, 'State: %s' % nav_data['ctrl_state'], (5, 15), font, font_size, (255, 255, 255))

                cv2.putText(image, 'Phi (+Roll Right/- Roll Left): %.0f' % nav_data['phi'], (5, 30), font, font_size, (255, 255, 255))
                cv2.putText(image, 'Psi (+Turn Right/- Turn Left): %.0f' % nav_data['psi'], (5, 45), font, font_size, (255, 255, 255))
                cv2.putText(image, 'Theta (+Up/-Down): %.0f' % nav_data['theta'], (5, 60), font, font_size, (255, 255, 255))

                cv2.putText(image, 'Altitude: %.0f' % nav_data['altitude'], (5, 75), font, font_size, (255, 255, 255))
                cv2.putText(image, 'vx/vy/vz: %.0f/%.0f/%.0f' % (nav_data['vx'], nav_data['vy'], nav_data['vz']), (5, 90), font, font_size, (255, 255, 255))

                cv2.putText(image, 'Battery: %.0f%%' % nav_data['battery'], (5, 105), font, font_size, (255, 255, 255))

                # if nav_data['battery'] == 0:
                #     print 'No battery left or not connected with correct WiFi!!'
                #     return

                cv2.putText(image, '# Frames (?): %.0f' % nav_data['num_frames'], (5, 120), font, font_size, (255, 255, 255))

            except Exception, e:
                print 'Exception happened while trying to access nav data:'
                logging.error(e, exc_info=True)

            cv2.rectangle(image, (50, 50), (100, 100), (0, 0, 255), 4, )

            # cv2.blur(image, (100, 100), (-1, -1))
            # mage = cv2.bilateralFilter(image, 9, 75, 75)

            cv2.imshow('Live Drone Cam (Jeronimo)', image)

            pressed_key = cv2.waitKey(1) & 0xFF

            if pressed_key != -1:
                if pressed_key == ord('q'):  # Q
                    running = False
                elif pressed_key == ord('w'):  # W
                    drone.move_forward()
                elif pressed_key == ord('s'):  # S
                    drone.move_backward()
                elif pressed_key == ord('a'):  # A
                    drone.move_left()
                elif pressed_key == ord('d'):  # D
                    drone.move_right()

                elif pressed_key == 0:  # Key Up
                    drone.move_up()
                elif pressed_key == 1:  # Key Down
                    drone.move_down()
                elif pressed_key == 2:  # Key Left
                    drone.turn_left()
                elif pressed_key == 3:  # Key Right
                    drone.turn_right()
                elif pressed_key == 32:  # Space
                    drone.land()
                elif pressed_key == 13:  # Enter
                    drone.takeoff()

                else:
                    print pressed_key

                    # sleep(0.01)

    except Exception, e:
        print 'Exception happened in main while loop:'
        logging.error(e, exc_info=True)

    cv2.destroyAllWindows()

    print("Shutting down...")
    drone.reset()
    drone.halt()
    print("Shut down.")


if __name__ == '__main__':
    main()
