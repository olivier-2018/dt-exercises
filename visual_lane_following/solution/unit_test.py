import numpy as np
import cv2
from matplotlib import pyplot as plt


class UnitTestELRH:
    # Test the estimate of the robot's lane-relative heading
    def __init__(self, estimate_lane_relative_heading):
        imgbgr_straight = cv2.imread('../images/visual_control/pic10.png')
        imgbgr_turn = cv2.imread('../images/visual_control/turn.png')

        # The image-to-ground homography associated with this image (Jacopo's)
        H = np.array([-4.137917960301845e-05, -0.00011445854191468058, -0.1595567007347241,
                      0.0008382870319844166, -4.141689222457687e-05, -0.2518201638170328,
                      -0.00023561657746150284, -0.005370140574116084, 0.9999999999999999])

        H = np.reshape(H, (3, 3))

        theta_hat, lines_left, lines_right = estimate_lane_relative_heading(H, imgbgr_straight)

        print('First Image: theta_hat: %.2f degrees' % (theta_hat*180/np.pi))
        fig = plt.figure(figsize=(20, 20))
        ax1 = fig.add_subplot(2, 2, 1)
        # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
        ax1.imshow(cv2.cvtColor(imgbgr_straight, cv2.COLOR_BGR2RGB))
        ax1.set_title('Input image'), ax1.set_xticks([]), ax1.set_yticks([])

        ax2 = fig.add_subplot(2, 2, 2)
        ax2.set_title('Lines Projected on Ground Plane'), ax2.set_xticks([]), ax2.set_yticks([]);
        if lines_left is not None:
            # Visualize the edges projected on to the ground plane
            for line in lines_left:
                [x1, y1, x2, y2] = line

                X = np.array([x1, x2])
                Y = np.array([y1, y2])

                # The ground reference frame has positive X up and positive Y left
                # So, for the sake of plotting we treat X as Y, and Y as -X
                ax2.plot(-Y, X, 'g-')

        if lines_right is not None:
            # Visualize the edges projected on to the ground plane
            for line in lines_right:
                [x1, y1, x2, y2] = line

                X = np.array([x1, x2])
                Y = np.array([y1, y2])

                # The ground reference frame has positive X up and positive Y left
                # So, for the sake of plotting we treat X as Y, and Y as -X
                ax2.plot(-Y, X, 'b-')

        theta_hat, lines_left, lines_right = estimate_lane_relative_heading(H, imgbgr_turn)

        print('Second Image: theta_hat: %.2f degrees' % (theta_hat * 180 / np.pi))
        ax3 = fig.add_subplot(2, 2, 3)
        # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
        ax3.imshow(cv2.cvtColor(imgbgr_turn, cv2.COLOR_BGR2RGB))
        ax3.set_title('Input image'), ax3.set_xticks([]), ax3.set_yticks([])

        ax4 = fig.add_subplot(2, 2, 4)
        ax4.set_title('Lines Projected on Ground Plane'), ax4.set_xticks([]), ax4.set_yticks([]);
        if lines_left is not None:
            # Visualize the edges projected on to the ground plane
            for line in lines_left:
                [x1, y1, x2, y2] = line

                X = np.array([x1, x2])
                Y = np.array([y1, y2])

                # The ground reference frame has positive X up and positivy Y left
                # So, for the sake of plotting we treat X as Y, and Y as -X
                ax4.plot(-Y, X, 'g-')

        if lines_right is not None:
            # Visualize the edges projected on to the ground plane
            for line in lines_right:
                [x1, y1, x2, y2] = line

                X = np.array([x1, x2])
                Y = np.array([y1, y2])

                # The ground reference frame has positive X up and positivy Y left
                # So, for the sake of plotting we treat X as Y, and Y as -X
                ax4.plot(-Y, X, 'b-')


def project_image_to_ground(H, x):
    """
        Args:
            H: The 3x3 image-to-ground plane homography (numpy.ndarray)
            x: An array of non-homogeneous image coordinates, one per column (numpy.ndarray)
        Returns:
            X: An array of non-homogeneous coordinates in the world (ground) frame, one per column (numpy.ndarray)
    """

    if x.shape[0] == 2:
        if x.ndim == 1:
            x = np.append(x, 1)
        else:
            x = np.vstack((x, np.ones((1, x.shape[1]))))

    X = H.dot(x)
    X = X / X[2, None]

    return X[0:2, ]
