from openai import OpenAI

client = OpenAI()

client.fine_tuning.jobs.create(
    training_file="file-D8aBgTaXbp3g42RcV2bTiKDK",
    model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    suffix="kios_ut_gen_v3",
    hyperparameters={"batch_size": 3, "learning_rate_multiplier": 0.5, "n_epochs": 1},
)
