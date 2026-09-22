#!/usr/bin/env python3
"""Simple TCP action server that loads an ACT model and serves actions.
Run in the lerobot conda env: python scripts/evaluation/act_action_server.py --checkpoint_path <path>

Protocol (JSON over TCP, one line per request/response):
  Client sends: {"images": {"left_wrist": "<base64>", "right_wrist": "<base64>", "front": "<base64>"}, "state": [0.1, 0.2, ...]}
  Server responds: {"actions": [[a1_1, a1_2, ...], [a2_1, ...], ...], "error": null}
"""
import argparse
import base64
import io
import json
import socketserver
import sys
import time
import traceback

import numpy as np
import torch
from PIL import Image


def load_act_policy(checkpoint_path: str, device: str = "cuda"):
    """Load ACT policy and pre/post processors from checkpoint."""
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    policy = ACTPolicy.from_pretrained(checkpoint_path)
    policy.to(device)
    policy.eval()

    preprocessor, postprocessor = make_pre_post_processors(policy.config, pretrained_path=checkpoint_path)

    return policy, preprocessor, postprocessor


def preprocess_observation(raw_obs: dict, preprocessor, device: str):
    """Convert raw observation dict to model input."""
    import torch

    # Decode images
    images = {}
    for cam_key in ["left_wrist", "right_wrist", "front"]:
        img_key = f"observation.images.{cam_key}"
        if cam_key in raw_obs.get("images", {}):
            img_b64 = raw_obs["images"][cam_key]
            img_bytes = base64.b64decode(img_b64)
            img = Image.open(io.BytesIO(img_bytes))
            img_np = np.array(img)  # (H, W, 3)
            img_tensor = torch.from_numpy(img_np).float() / 255.0
            images[img_key] = img_tensor.permute(2, 0, 1)  # (C, H, W)

    # State
    state = raw_obs.get("state", [0.0] * 12)
    state_tensor = torch.tensor(state, dtype=torch.float32)

    # Build observation dict
    obs_dict = {
        "observation.state": state_tensor,
        **images,
    }

    # Apply preprocessor (handles batching internally)
    batch = preprocessor(obs_dict)
    batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v for k, v in batch.items()}

    return batch


def postprocess_actions(action_tensor, postprocessor):
    """Convert model output to raw action values."""
    # action_tensor: (1, chunk_size, action_dim)
    actions = action_tensor[0]  # (chunk_size, action_dim)

    # PolicyAction is just a torch.Tensor
    result = postprocessor(actions)
    actions = result.cpu().numpy() if isinstance(result, torch.Tensor) else result["action"].cpu().numpy()

    return actions.tolist()


class ActionHandler(socketserver.StreamRequestHandler):
    policy = None
    preprocessor = None
    postprocessor = None
    device = "cuda"

    def handle(self):
        print(f"[Server] Client connected: {self.client_address}")
        while True:
            try:
                line = self.rfile.readline()
                if not line:
                    break

                request = json.loads(line.decode("utf-8").strip())

                # Preprocess
                batch = preprocess_observation(request, self.preprocessor, self.device)

                # Inference
                with torch.inference_mode():
                    action_tensor = self.policy.predict_action_chunk(batch)

                # Postprocess
                actions = postprocess_actions(action_tensor, self.postprocessor)

                response = json.dumps({"actions": actions, "error": None}) + "\n"
                self.wfile.write(response.encode("utf-8"))
                self.wfile.flush()

            except Exception as e:
                traceback.print_exc()
                error_response = json.dumps({"actions": None, "error": str(e)}) + "\n"
                try:
                    self.wfile.write(error_response.encode("utf-8"))
                    self.wfile.flush()
                except Exception:
                    break
                break

        print(f"[Server] Client disconnected: {self.client_address}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint_path", type=str, required=True)
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args()

    print(f"[Server] Loading ACT model from {args.checkpoint_path}...")
    policy, preprocessor, postprocessor = load_act_policy(args.checkpoint_path, args.device)
    print("[Server] Model loaded.")

    ActionHandler.policy = policy
    ActionHandler.preprocessor = preprocessor
    ActionHandler.postprocessor = postprocessor
    ActionHandler.device = args.device

    server = socketserver.ThreadingTCPServer((args.host, args.port), ActionHandler)
    print(f"[Server] Listening on {args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("[Server] Shutting down.")
        server.shutdown()


if __name__ == "__main__":
    main()
