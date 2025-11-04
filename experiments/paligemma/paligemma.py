import os
from dotenv import load_dotenv
from huggingface_hub import login

load_dotenv()

login(token=os.getenv("HF_AUTH_TOKEN"))

# Use a pipeline as a high-level helper
from transformers import pipeline

pipe = pipeline("image-text-to-text", model="google/paligemma-3b-pt-896")

# Load model directly
from transformers import AutoProcessor, AutoModelForVision2Seq

processor = AutoProcessor.from_pretrained("google/paligemma-3b-pt-896")
model = AutoModelForVision2Seq.from_pretrained("google/paligemma-3b-pt-896")