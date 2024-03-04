from openai import OpenAI
import os

openai_client = OpenAI()

file_name = "finetuning_messages_20240303.jsonl"

training_file = openai_client.files.create(
    file=open(
        os.path.join(os.path.dirname(__file__), file_name),
        "rb",
    ),
    purpose="fine-tune",
)

# # Wait while the file is processed
# status = openai_client.files.retrieve("file-EhbEPGHSCMj1bD6TGi6WQMxT").status


# start_time = time.time()
# while status != "processed":
#     print(f"Status=[{status}]... {time.time() - start_time:.2f}s", end="\r", flush=True)
#     time.sleep(5)
#     status = openai.File.retrieve(training_file.id).status
# print(f"File {training_file.id} ready after {time.time() - start_time:.2f} seconds.")

# from openai import OpenAI

# client = OpenAI()

# client.fine_tuning.jobs.create(training_file="UT_gen_test", model="gpt-3.5-turbo")
