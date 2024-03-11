from openai import OpenAI

client = OpenAI()

# List 10 fine-tuning jobs
print(client.fine_tuning.jobs.list(limit=10))
