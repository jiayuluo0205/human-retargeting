import os
import cv2
import yaml
import numpy as np
from pupil_apriltags import Detector

if __name__ == "__main__":  # noqa

    test_images_path = "3rdparty/apriltags/test"
    with open(test_images_path + "/test_info.yaml") as stream:
        parameters = yaml.load(stream, Loader=yaml.FullLoader)

    visualization = True

    at_detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    # TEST WITH THE SAMPLE IMAGE

    print("\n\nTESTING WITH A SAMPLE IMAGE")

    img = cv2.imread(
        test_images_path + "/" + parameters["sample_test"]["file"], cv2.IMREAD_GRAYSCALE
    )  # NOTE: the image should be grayscale
    cameraMatrix = np.array(parameters["sample_test"]["K"]).reshape((3, 3))
    camera_params = (
        cameraMatrix[0, 0],
        cameraMatrix[1, 1],
        cameraMatrix[0, 2],
        cameraMatrix[1, 2],
    )

    if visualization:
        cv2.imshow("Original image", img)

    tags = at_detector.detect(
        img, True, camera_params, parameters["sample_test"]["tag_size"]
    )  # NOTE: 
    print(tags)

    color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(
                color_img,
                tuple(tag.corners[idx - 1, :].astype(int)),  
                tuple(tag.corners[idx, :].astype(int)),
                (0, 255, 0),
            )  # NOTE: draw the tag edges

        cv2.putText(
            color_img,
            str(tag.tag_id),
            org=(
                tag.corners[0, 0].astype(int) + 10,
                tag.corners[0, 1].astype(int) + 10,
            ),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.8,
            color=(0, 0, 255),
        )

    if visualization:
        cv2.imshow("Detected tags", color_img)

        k = cv2.waitKey(0)
        if k == 27:  # wait for ESC key to exit
            cv2.destroyAllWindows()

    # TEST WITH THE ROTATION IMAGES

    import time

    print("\n\nTESTING WITH ROTATION IMAGES")

    time_num = 0
    time_sum = 0

    image_names = parameters["rotation_test"]["files"]
    from loguru import logger as lgr

    for image_name in image_names:
        print("Testing image ", image_name)
        ab_path = test_images_path + "/" + image_name
        if not os.path.isfile(ab_path):
            lgr.warning(f"File {ab_path} not found")
            continue
        groundtruth = float(
            image_name.split("_")[-1].split(".")[0]
        )  # name of test image should be set to its groundtruth

        parameters["rotation_test"]["rotz"] = groundtruth
        cameraMatrix = np.array(parameters["rotation_test"]["K"]).reshape((3, 3))
        camera_params = (
            cameraMatrix[0, 0],
            cameraMatrix[1, 1],
            cameraMatrix[0, 2],
            cameraMatrix[1, 2],
        )

        img = cv2.imread(ab_path, cv2.IMREAD_GRAYSCALE)

        start = time.time()
        tags = at_detector.detect(
            img, True, camera_params, parameters["rotation_test"]["tag_size"]
        )

        time_sum += time.time() - start
        time_num += 1

        print(
            tags[0].pose_t,
            parameters["rotation_test"]["posx"],
            parameters["rotation_test"]["posy"],
            parameters["rotation_test"]["posz"],
        )
        print(
            tags[0].pose_R,
            parameters["rotation_test"]["rotx"],
            parameters["rotation_test"]["roty"],
            parameters["rotation_test"]["rotz"],
        )

    print("AVG time per detection: ", time_sum / time_num)

    # TEST WITH MULTIPLE TAGS IMAGES

    print("\n\nTESTING WITH MULTIPLE TAGS IMAGES")

    time_num = 0
    time_sum = 0

    image_names = parameters["multiple_tags_test"]["files"]
    for image_name in image_names:
        print("Testing image ", image_name)
        ab_path = test_images_path + "/" + image_name
        if not os.path.isfile(ab_path):
            lgr.warning(f"File {ab_path} not found")
            continue

        cameraMatrix = np.array(parameters["multiple_tags_test"]["K"]).reshape(
            (3, 3)
        )
        camera_params = (
            cameraMatrix[0, 0],
            cameraMatrix[1, 1],
            cameraMatrix[0, 2],
            cameraMatrix[1, 2],
        )

        img = cv2.imread(ab_path, cv2.IMREAD_GRAYSCALE)

        start = time.time()
        tags = at_detector.detect(
            img, True, camera_params, parameters["multiple_tags_test"]["tag_size"]
        )
        time_sum += time.time() - start
        time_num += 1
        tag_ids = [tag.tag_id for tag in tags]
        print(len(tags), " tags found: ", tag_ids)

        color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(
                    color_img,
                    tuple(tag.corners[idx - 1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0),
                )

            cv2.putText(
                color_img,
                str(tag.tag_id),
                org=(
                    tag.corners[0, 0].astype(int) + 10,
                    tag.corners[0, 1].astype(int) + 10,
                ),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255),
            )

        if visualization:
            cv2.imshow("Detected tags for " + image_name, color_img)

            k = cv2.waitKey(0)
            if k == 27:  # wait for ESC key to exit
                cv2.destroyAllWindows()

    print("AVG time per detection: ", time_sum / time_num)
