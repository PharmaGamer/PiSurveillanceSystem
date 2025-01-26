import os
import time
import json
import dropbox

# Load Dropbox access token from config.json
def load_config():
    with open("config.json", "r") as f:
        return json.load(f)

config = load_config()
DROPBOX_ACCESS_TOKEN = config["DROPBOX_ACCESS_TOKEN"]

# Initialize Dropbox client
dbx = dropbox.Dropbox(DROPBOX_ACCESS_TOKEN)

# Define local directory to monitor and Dropbox folder
LOCAL_DIRECTORY = "/var/www/html/videos"
DROPBOX_FOLDER = "/PiVideoBackup"

# Function to check if a file's size is stable
def file_is_finalized(file_path, wait_time=2):
    """
    Checks if the file size remains constant over a period of time.
    :param file_path: Path to the file to check.
    :param wait_time: Time in seconds to wait and check file size stability.
    :return: True if the file size is stable, False otherwise.
    """
    try:
        initial_size = os.path.getsize(file_path)
        time.sleep(wait_time)
        current_size = os.path.getsize(file_path)
        return initial_size == current_size
    except FileNotFoundError:
        # File may have been deleted/moved during the check
        return False

# Function to upload a file to Dropbox
def upload_to_dropbox(local_path, dropbox_path):
    with open(local_path, "rb") as f:
        print(f"Uploading {local_path} to Dropbox as {dropbox_path}...")
        try:
            dbx.files_upload(f.read(), dropbox_path, mode=dropbox.files.WriteMode("overwrite"))
            print(f"Uploaded: {local_path}")
        except Exception as e:
            print(f"Error uploading {local_path}: {e}")

# Function to monitor the directory and upload new files
def monitor_and_upload():
    uploaded_files = set()

    while True:
        try:
            # Scan local directory for files
            current_files = set(os.listdir(LOCAL_DIRECTORY))

            # Identify new files
            new_files = current_files - uploaded_files

            for file_name in new_files:
                local_path = os.path.join(LOCAL_DIRECTORY, file_name)
                dropbox_path = f"{DROPBOX_FOLDER}/{file_name}"

                # Check if the file is finalized
                if file_is_finalized(local_path):
                    # Upload the finalized file to Dropbox
                    upload_to_dropbox(local_path, dropbox_path)
                    # Add file to uploaded list
                    uploaded_files.add(file_name)
                else:
                    print(f"File {file_name} is still being written. Retrying...")

            # Wait for a short interval before scanning again
            time.sleep(5)

        except KeyboardInterrupt:
            print("Monitoring stopped by user.")
            break
        except Exception as e:
            print(f"An error occurred: {e}")
            time.sleep(5)

if __name__ == "__main__":
    monitor_and_upload()
