from dataclasses import dataclass
from typing import List, Optional, Dist, Any


@dataclass
class MiosResponse:
    result: dict
    error: Optional[str]
