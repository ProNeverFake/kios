from openai import OpenAI

openai_client = OpenAI()

status = openai_client.files.retrieve("file-EhbEPGHSCMj1bD6TGi6WQMxT").status

print(status)
