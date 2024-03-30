from semantic_router import Route
from semantic_router.encoders import CohereEncoder, OpenAIEncoder
from semantic_router.layer import RouteLayer

from getpass import getpass

from dotenv import load_dotenv
import os
import json

load_dotenv()

data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())


class KiosRouterFactory:
    encoder: OpenAIEncoder | CohereEncoder = None
    utterance_dictionary: dict[str, list[str]] = None

    def __init__(self, encoder="openai") -> None:
        # initialize the encoder
        if encoder == "openai":
            os.environ["OPENAI_API_KEY"] = os.getenv("OPENAI_API_KEY") or getpass(
                "OpenAI API Key not found, please enter it here or stop here and check your environment variables:"
            )
            self.encoder = OpenAIEncoder(score_threshold=0.5)
        elif encoder == "cohere":
            os.environ["COHERE_API_KEY"] = os.getenv("COHERE_API_KEY") or getpass(
                "Cohere API Key not found, please enter it here or stop here and check your environment variables:"
            )
            self.encoder = (
                CohereEncoder()
            )  # * you may need to adjust the score_threshold
        else:
            raise ValueError(f"Encoder {encoder} not supported")

        # fetch the utterances
        self.utterance_dictionary = json.load(
            open(os.path.join(data_dir, "router_utterances.json"))
        )

    def create_router_layer(self, route_names: list[str]) -> RouteLayer:
        if len(route_names) <= 1:
            raise ValueError(
                f"It makes no sense to create a router with less than one route! Please provide at least two routes."
            )
        return RouteLayer(
            encoder=self.encoder,
            routes=self._fetch_utterance(route_names),
        )

    def _fetch_utterance(self, route_names: list[str]) -> list[Route]:
        # check first
        for name in route_names:
            if name not in self.utterance_dictionary.keys():
                raise ValueError(f"Utterance {name} not found in the data file!")

        # make the routes
        return [
            Route(
                name=name,
                utterances=self.utterance_dictionary[name],
            )
            for name in route_names
        ]


def test():
    factory = KiosRouterFactory()
    router = factory.create_router_layer(["finish", "instruction"])

    from pprint import pprint

    pprint(router("can you insert the shaft1 into hole 1?"))


if __name__ == "__main__":
    test()
