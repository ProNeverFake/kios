from openai import OpenAI

client = OpenAI()

client.fine_tuning.jobs.create(
    training_file="file-ZduAH3vCfTfZD6fYA8VeHOuI",
    model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-onestep-v1:9DprEvbP",
    suffix="kios_onestep_v2",
    # hyperparameters={"batch_size": 3, "learning_rate_multiplier": 0.5, "n_epochs": 1},
    hyperparameters={"batch_size": 1, "learning_rate_multiplier": 0.1, "n_epochs": 1},
)
