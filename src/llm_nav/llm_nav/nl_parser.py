import json
import os
import requests
import re
import argparse

# Load API tokens from environment variables
HF_TOKEN = os.getenv("HF_API_TOKEN")
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not HF_TOKEN:
    raise RuntimeError("Hugging Face API token not found. Please set HF_API_TOKEN in your environment.")

# Hugging Face Model URL
HF_MODEL_NAME = "HuggingFaceH4/zephyr-7b-beta"
HF_API_URL = f"https://api-inference.huggingface.co/models/{HF_MODEL_NAME}"

hf_headers = {
    "Authorization": f"Bearer {HF_TOKEN}",
    "Content-Type": "application/json"
}


def call_openrouter_fallback(prompt: str) -> str:
    if not OPENROUTER_API_KEY:
        print("OpenRouter API key not found. Skipping fallback.")
        return ""

    headers = {
        "Authorization": f"Bearer {OPENROUTER_API_KEY}",
        "Content-Type": "application/json",
        "HTTP-Referer": "https://yourdomain.com",
        "X-Title": "RobotPlanner"
    }

    payload = {
        "model": "deepseek/deepseek-r1:free",
        "messages": [
            {
                "role": "system",
                "content": "You are a home assistant planner for a robot. Convert the user instruction into a full action plan. Each action must be a list with two elements: ['ActionType', 'Target']."
            },
            {"role": "user", "content": prompt}
        ],
        "temperature": 0.1,
        "max_tokens": 300
    }

    try:
        response = requests.post("https://openrouter.ai/api/v1/chat/completions", headers=headers, json=payload, timeout=20)
        response.raise_for_status()
        print("Full OpenRouter Response: ", response.text)
        content = response.json()['choices'][0]['message']['content']
        print("Raw OpenRouter response:\n", content)
        return content
    except Exception as e:
        print(f"OpenRouter fallback failed: {e}")
        return ""


def call_gemini(prompt: str) -> str:
    if not GEMINI_API_KEY:
        raise RuntimeError("Gemini API key not found. Please set GEMINI_API_KEY in your environment.")

    url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent"
    headers = {
        "Content-Type": "application/json"
    }
    payload = {
        "contents": [
            {
                "parts": [
                    {
                        "text": prompt
                    }
                ]
            }
        ]
    }

    try:
        response = requests.post(f"{url}?key={GEMINI_API_KEY}", headers=headers, json=payload, timeout=15)
        response.raise_for_status()
        content = response.json()["candidates"][0]["content"]["parts"][0]["text"]
        print("Raw Gemini response:\n", content)
        return content
    except Exception as e:
        print(f"Gemini call failed: {e}")
        raise


def parse_output(text: str):
    match = re.search(r"\[\s*\[.*?\]\s*\]", text, re.DOTALL)
    if not match:
        print("No valid action list found.")
        return []

    try:
        return eval(match.group(0))
    except Exception as e:
        print(f"Error evaluating action list: {e}")
        return []


def llm_generate_plan(user_input: str) -> list:
    prompt = f"""You are a home assistant planner for a robot. Convert the user instruction into a full action plan.
Each action must be a list with two elements: ['ActionType', 'Target'].

Examples:
User: 'Bring coffee to the bedroom'
[
    ["Navigate", "kitchen"],
    ["Make", "coffee"],
    ["Navigate", "bedroom"]
]

User: 'Turn on the lamp in my bedroom'
[
    ["Navigate", "bedroom"],
    ["Turn on", "lamp"]
]

User: 'Bring me my computer to the bedroom'
[
    ["Navigate", "office"],
    ["Take", "computer"],
    ["Navigate", "bedroom"]
]

User: 'Turn off the TV and come back to my bedroom'
[
    ["Navigate", "living room"],
    ["Turn off", "TV"],
    ["Navigate", "bedroom"]
]

User: 'Clean the kitchen and bring me the dishes'
[
    ["Navigate", "kitchen"],
    ["Clean", "kitchen"],
    ["Take", "dishes"],
    ["Navigate", "living room"]
]

Now generate a plan for the following:
User: '{user_input}'
"""

    try:
        model_output = call_gemini(prompt)
        plan = parse_output(model_output)
    except Exception:
        try:
            payload = {
                "inputs": prompt,
                "parameters": {
                    "temperature": 0.1,
                    "max_new_tokens": 200
                }
            }
            response = requests.post(HF_API_URL, headers=hf_headers, json=payload, timeout=15)
            response.raise_for_status()
            model_output = response.json()[0]["generated_text"]
            plan = parse_output(model_output)
        except Exception as e:
            print(f"Hugging Face call failed or irrelevant: {e}")
            print("Attempting OpenRouter fallback...")
            model_output = call_openrouter_fallback(prompt)
            if not model_output:
                return []
            plan = parse_output(model_output)

    cleaned_plan = []
    for pair in plan:
        if isinstance(pair, list) and len(pair) >= 2:
            action_type = pair[0].strip().capitalize()
            target = pair[1].strip()
            cleaned_plan.append([action_type, target])
    return cleaned_plan


def main():
    parser = argparse.ArgumentParser(description="Generate action plan from natural language instruction.")
    parser.add_argument('--user_input', type=str, default="Bring coffee to my bedroom", help='User instruction to process')
    args = parser.parse_args()

    user_input = args.user_input
    plan = llm_generate_plan(user_input)

    print(f"User: '{user_input}'")
    if not plan:
        print("No valid plan was generated.")
    else:
        print(f"Generated Plan: {plan}\n")


if __name__ == "__main__":
    main()
