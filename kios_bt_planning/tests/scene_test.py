from kios_scene.scene_factory import SceneFactory


def test_scene_factory():
    sf = SceneFactory()

    # scene_json = {
    #     "reference_objects": [
    #         {
    #             "object_name": "base",
    #             "source": "pre-defined",  # can be mios, vision, or pre-defined
    #             "joint_pose": [0, 0, 0, 0, 0, 0, 0],
    #             "O_T_EE": [
    #                 [1, 0, 0, 0.5],
    #                 [0, 1, 0, 0.5],
    #                 [0, 0, 1, 0.5],
    #                 [0, 0, 0, 1],
    #             ],
    #             "reference_object": None,
    #         },
    #         {
    #             "object_name": "another_base",
    #             "source": "mios",  # means the object is from mios mongodb, usually a location that is taught.
    #             "key": "another_base",  # the key to find the object in the mongodb
    #         },
    #         {
    #             "object_name": "a_third_base",
    #             "source": "vision",  # means the object is from vision detection
    #             "key": "a_third_base",  # the key to find the object in the detection list. the location is preprocessed before being accessed.
    #         },
    #     ],
    #     "relative_objects": {
    #         "shaft1_hole": {
    #             "reference_object": "base",
    #             "relative_joint_pose": None,
    #             "relative_HT": [
    #                 [1, 0, 0, 0],
    #                 [0, 1, 0, 0],
    #                 [0, 0, 1, -0.2],
    #                 [0, 0, 0, 1],
    #             ],
    #             "relative_cartesian_pose": None,
    #         },
    #         "shaft2": {
    #             "reference_object": "base",
    #             "relative_joint_pose": [0, 0, 0, 0, 0, 0, 0],
    #             "relative_HT": None,
    #             "relative_cartesian_pose": [0, 0, 0, 0, 0, 0.2],
    #         },
    #     },
    # }

    scene_json = {
        "reference_objects": [
            {
                "object_name": "base",
                "source": "pre-defined",  # can be mios, vision, or pre-defined
                "joint_pose": [0, 0, 0, 0, 0, 0, 0],
                "O_T_EE": [
                    [1, 0, 0, 0.5],
                    [0, 1, 0, 0.5],
                    [0, 0, 1, 0.5],
                    [0, 0, 0, 1],
                ],
                "reference_object": None,
            },
            {
                "object_name": "parallel_box1",
                "source": "mios",  # means the object is from mios mongodb, usually a location that is taught.
                "key": "parallel_box1",  # the key to find the object in the mongodb
            },
            {
                "object_name": "inward_craw",
                "source": "mios",
                "key": "inward_craw",
            },
            {
                "object_name": "outward_craw",
                "source": "mios",
                "key": "outward_craw",
            },
        ],
        "relative_objects": {
            # "shaft1_hole": {
            #     "reference_object": "base",
            #     "relative_joint_pose": None,
            #     "relative_HT": [
            #         [1, 0, 0, 0],
            #         [0, 1, 0, 0],
            #         [0, 0, 1, -0.2],
            #         [0, 0, 0, 1],
            #     ],
            #     "relative_cartesian_pose": None,
            # },
            # "shaft2": {
            #     "reference_object": "base",
            #     "relative_joint_pose": [0, 0, 0, 0, 0, 0, 0],
            #     "relative_HT": None,
            #     "relative_cartesian_pose": [0, 0, 0, 0, 0, 0.2],
            # },
        },
    }
