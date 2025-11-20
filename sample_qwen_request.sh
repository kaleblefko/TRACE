#!/bin/bash

API_URL="http://127.0.0.1:11434/v1/chat/completions"
MODEL="qwen3-vl:8b"
# MODEL="gemma3"

read -rp "Enter path to local image file: " IMG_PATH

if [[ ! -f "$IMG_PATH" ]]; then
    echo "[ERROR] File not found: $IMG_PATH"
    exit 1
fi

# Extract extension 
EXT="${IMG_PATH##*.}"
EXT=$(echo "$EXT" | tr '[:upper:]' '[:lower:]')

# Base64 encode image
B64_RAW=$(base64 -i "$IMG_PATH")
# Escape newlines for JSON
B64_ESCAPED=$(echo "$B64_RAW" | tr -d '\n')

# Build JSON payload
read -r -d '' JSON_PAYLOAD <<EOF
{
  "model": "$MODEL",
  "messages": [
    {
      "role": "system",
      "content": "You are a helpful AI assistant."
    },
    {
      "role": "user",
      "content": [
        { "type": "text", "text": "Describe this image." },
        {
          "type": "image_url",
          "image_url": {
            "url": "data:image/$EXT;base64,$B64_ESCAPED"
          }
        }
      ]
    }
  ],
  "temperature": 0.2,
  "max_tokens": 512
}
EOF

echo
echo ">>> Sending request to $API_URL ..."
echo

curl -X POST "$API_URL" \
    -H "Content-Type: application/json" \
    -d "$JSON_PAYLOAD"

echo
