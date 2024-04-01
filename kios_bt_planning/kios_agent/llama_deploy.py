"""
Need azure cli and azure ai sdk
Currently problematic. 
last error:
Traceback (most recent call last):
  File "/home/blackbird/kios_workspace/src/kios/kios_bt_planning/kios_agent/llama_deploy.py", line 23, in <module>
    client.deployments.create_or_update(deployment)
  File "/home/blackbird/miniconda3/envs/kios/lib/python3.10/site-packages/azure/core/tracing/decorator.py", line 78, in wrapper_use_tracer
    return func(*args, **kwargs)
  File "/home/blackbird/miniconda3/envs/kios/lib/python3.10/site-packages/azure/ai/ml/_telemetry/activity.py", line 275, in wrapper
    return f(*args, **kwargs)
  File "/home/blackbird/miniconda3/envs/kios/lib/python3.10/site-packages/azure/ai/resources/operations/_deployment_operations.py", line 209, in create_or_update
    family_usage = filtered_usage_info[family]
KeyError: 'StandardNCADSA100v4Family'
"""

# ! NOT IN USE NOW. THE LLAMA 70B MODEL IS DEPLOYED AS PAY-AS-YOU-GO SERVICE IN AZURE.
from azure.ai.resources.client import AIClient
from azure.ai.resources.entities.deployment import Deployment
from azure.ai.resources.entities.models import PromptflowModel
from azure.identity import DefaultAzureCredential

credential = DefaultAzureCredential()

client = AIClient(
    credential=credential,
    subscription_id="74f1dc98-0762-4789-a9d1-8c145058b546",
    resource_group_name="blackbird_group",
    project_name="llama2-kios",
)

model_id = "azureml://registries/azureml-meta/models/Llama-2-70b/versions/15"
deployment_name = "llam270b-deployment"

deployment = Deployment(
    name=deployment_name,
    model=model_id,
)

client.deployments.create_or_update(deployment)
