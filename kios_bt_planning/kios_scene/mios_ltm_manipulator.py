import os
import json
import logging
from kios_scene.mongodb_interface import MongoDBInterface
from kios_robot.data_types import MiosObject
from bson import ObjectId

"""
this helps with fast deployment of the task scene
"""


# json serializer for ObjectId
def json_serial(obj):
    """JSON serializer for objects not serializable by default json code"""
    if isinstance(obj, ObjectId):
        return str(obj)
    raise TypeError("Type %s not serializable" % type(obj))


class LangTermMemoryManipulator:
    mongodb_interface: MongoDBInterface = None
    backup_dir: str = None

    def __init__(self, backup_dir: str = None) -> None:
        self.mongodb_interface = MongoDBInterface()
        if backup_dir is not None:
            self.backup_dir = backup_dir
        else:
            if not os.path.exists(os.path.join(os.getcwd(), "mios_memory_backups")):
                os.makedirs(os.path.join(os.getcwd(), "mios_memory_backups"))
            self.backup_dir = os.path.join(os.getcwd(), "mios_memory_backups")

    def backup_mios_environment(self, backup_name: str) -> None:
        documents = self.mongodb_interface.fetch_all_mios_objects()
        backup_file_path = os.path.join(self.backup_dir, f"{backup_name}.json")
        if os.path.exists(backup_file_path):
            logging.warning
            print(
                "\033[91mThe backup file with name '{}' already exists. Type yes to overwrite. Type c to cancel the operation.".format(
                    backup_name
                )
            )
            user_input = input()
            if user_input == "yes":
                print(
                    "\033[92mOverwriting the backup file '{}'".format(backup_file_path)
                )
                with open(backup_file_path, "w") as backup_file:
                    json.dump(documents, backup_file, default=json_serial, indent=4)
            elif user_input == "c":
                print("\033[92mOperation canceled.")
                return
            else:
                print("\033[93mInvalid input. Operation canceled.")
                return
        else:
            print(
                "\033[92mWrite the mios objects to a new backup file '{}'".format(
                    backup_file_path
                )
            )
            with open(backup_file_path, "w") as backup_file:
                json.dump(documents, backup_file, default=json_serial, indent=4)

    def clear_mios_environment(self) -> None:
        self.mongodb_interface.erase_all_mios_objects()

    def show_backups(self) -> None:
        print("The existing backups are:")
        for file in os.listdir(self.backup_dir):
            backup_name = file.split(".")[0]
            print("- {}".format(backup_name))

    def update_to_mios_environment(self, backup_name: str) -> None:
        """
        update objects.
        if not exist, insert, if exist, update
        """
        backup_file_path = os.path.join(self.backup_dir, f"{backup_name}.json")
        if not os.path.exists(backup_file_path):
            print(
                "\033[93mThe backup file '{}' does not exist.".format(backup_file_path)
            )
            return

        print(
            "\033[91mPlease make sure you want to execute this operation. Type yes to confirm. Type c to cancel the operation."
        )
        user_input = input()
        if user_input == "yes":
            with open(backup_file_path, "r") as backup_file:
                documents = json.load(backup_file)
                self.mongodb_interface.update_mios_objects(documents)
                print(
                    "\033[92mUpdated the mios objects from '{}'".format(
                        backup_file_path
                    )
                )
        elif user_input == "c":
            print("\033[92mOperation canceled.")
            return
        else:
            print("\033[93mInvalid input. Operation canceled.")
            return

    def restore_to_mios_environment(self, backup_name: str) -> None:
        """
        erase all and inject the backup to mongodb
        """
        backup_file_path = os.path.join(self.backup_dir, f"{backup_name}.json")
        if not os.path.exists(backup_file_path):
            print(
                "\033[93mThe backup file '{}' does not exist.".format(backup_file_path)
            )
            return

        print(
            "\033[91mPlease make sure you want to execute this operation. Type yes to confirm. Type c to cancel the operation."
        )
        user_input = input()
        if user_input == "yes":
            with open(backup_file_path, "r") as backup_file:
                documents = json.load(backup_file)
                self.mongodb_interface.erase_all_mios_objects()
                self.mongodb_interface.inject_mios_objects(documents)
                print(
                    "\033[92mRestored the mios objects from '{}'".format(
                        backup_file_path
                    )
                )
                print("\033[0m")  # Reset color
        elif user_input == "c":
            print("\033[92mOperation canceled.")
            print("\033[0m")  # Reset color
            return
        else:
            print("\033[93mInvalid input. Operation canceled.")
            print("\033[0m")  # Reset color
            return
