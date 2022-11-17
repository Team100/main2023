import cv2
import numpy as np


def draw_cube(overlay, camera_params, tag_size, rvec, tvec, z_sign=1):

    opoints = (
        np.array(
            [
                [-1, -1, 0],
                [1, -1, 0],
                [1, 1, 0],
                [-1, 1, 0],
                [-1, -1, 1 * z_sign],
                [1, -1, 1 * z_sign],
                [1, 1, 1 * z_sign],
                [-1, 1, 1 * z_sign],
            ]
        )
        * 0.5
        * tag_size
    )

    edges = np.array(
        [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4],
        ]
    )

    fx, fy, cx, cy = camera_params
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dcoeffs = np.zeros(5)
    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
    ipoints = np.round(ipoints).astype(int)
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (255, 0, 0), 1, 16)


def draw_pose(overlay, camera_params, tag_size, rvec, tvec, z_sign=1):
    opoints = (
        np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [1, -1, -1 * z_sign]])
        * 0.5
        * tag_size
    )

    fx, fy, cx, cy = camera_params

    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dcoeffs = np.zeros(5)
    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
    ipoints = np.round(ipoints).astype(int)
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    cv2.line(overlay, ipoints[0], ipoints[1], (0, 0, 255), 2)
    cv2.line(overlay, ipoints[1], ipoints[2], (0, 255, 0), 2)
    cv2.line(overlay, ipoints[1], ipoints[3], (255, 0, 0), 2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(overlay, "X", ipoints[0], font, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(overlay, "Y", ipoints[2], font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(overlay, "Z", ipoints[3], font, 0.5, (255, 0, 0), 2, cv2.LINE_AA)


def draw_result(image, camera_params, tag_size, result):
    for r in result:
        if r.hamming > 0:
            continue
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)

        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

        tagFamily = r.tag_family.decode("utf-8")
        tagId = r.tag_id
        cv2.putText(
            image,
            f"id {tagId}",
            (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1,
        )
        # print(f"[INFO] tag family: {tagFamily}")
        # print(f"[INFO] tag id: {tagId}")

        # draw_pose(image, camera_params=camera_params, tag_size=tag_size, rvec=r.pose_R, tvec=r.pose_t, z_sign=1)
        draw_cube(
            image,
            camera_params=camera_params,
            tag_size=tag_size,
            rvec=r.pose_R,
            tvec=r.pose_t,
            z_sign=-1,
        )
