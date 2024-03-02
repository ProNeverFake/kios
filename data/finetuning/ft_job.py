from openai import OpenAI

client = OpenAI()

client.fine_tuning.jobs.create(
    training_file="file-EhbEPGHSCMj1bD6TGi6WQMxT",
    model="gpt-3.5-turbo",
)
