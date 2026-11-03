# 📓 Physical-AI Training Notebooks

[![Kaggle](https://img.shields.io/badge/Platform-Kaggle%202x%20T4-20BEFF?logo=kaggle&logoColor=white)](https://www.kaggle.com/)
[![Hugging Face Dataset](https://img.shields.io/badge/%F0%9F%A4%97%20Dataset-lekiwi__chess__v7-blue.svg)](https://huggingface.co/datasets/DuyKhongCay/lekiwi_chess_v7)
[![Hugging Face Model](https://img.shields.io/badge/%F0%9F%A4%97%20Model-lekiwi__chess__smolvla-yellow.svg)](https://huggingface.co/DuyKhongCay/lekiwi_chess_smolvla)
[![Framework](https://img.shields.io/badge/Framework-Hugging%20Face%20LeRobot%20v0.6+-FFD21E)](https://github.com/huggingface/lerobot)

This directory contains standalone Jupyter notebooks for reproducing the cloud training and fine-tuning pipelines of LeKiwi's Physical-AI manipulation policies.

---

## 📋 Available Notebooks

| Notebook | Target Policy | Input Dataset | Output Checkpoint | Compute Requirements |
| :--- | :--- | :--- | :--- | :--- |
| [`lerobot_smolvla_training_kaggle.ipynb`](lerobot_smolvla_training_kaggle.ipynb) | SmolVLA (500M) | [`DuyKhongCay/lekiwi_chess_v7`](https://huggingface.co/datasets/DuyKhongCay/lekiwi_chess_v7) | [`DuyKhongCay/lekiwi_chess_smolvla`](https://huggingface.co/DuyKhongCay/lekiwi_chess_smolvla) | Kaggle `2x Tesla T4` (16 GB each), ~8.5h |

---

## 🚀 How to Run on Kaggle (Step-by-Step)

### 1. Create a New Kaggle Notebook
1. Go to [Kaggle Code](https://www.kaggle.com/code) and click **"New Notebook"**.
2. Go to **File $\to$ Import Notebook** and upload [`lerobot_smolvla_training_kaggle.ipynb`](lerobot_smolvla_training_kaggle.ipynb).

### 2. Configure Hardware Accelerator
In the right-hand **Notebook settings** panel:
* **Accelerator**: Select **`GPU T4 x2`** (Dual NVIDIA Tesla T4 16GB).
* **Internet**: Turn **`ON`** (required for pip packages, downloading base models, and uploading checkpoints to Hugging Face).

### 3. Add Hugging Face Secret Token
1. In the Kaggle top menu, click **Add-ons $\to$ Secrets**.
2. Add a new secret:
   * **Label**: `HF_TOKEN`
   * **Value**: Your Hugging Face user access token (with `write` permission).
3. Ensure the checkbox next to `HF_TOKEN` is checked to attach it to the notebook.

### 4. Execute the Training Pipeline
Run all notebook cells sequentially:
* **Cell 1**: Installs `lerobot[training,dataset]>=0.6.0`, `accelerate`, and `ffmpeg`. Verifies dual GPU detection.
* **Cell 2**: Authenticates to Hugging Face Hub using the `HF_TOKEN` secret.
* **Cell 3**: Builds and registers the custom plugin `lerobot_policy_chess_plugins`:
  - Registers `ChessGoalNoiseProcessorStep` (`chess_goal_noise`) into LeRobot's processor registry.
  - Injects Gaussian perturbation ($\sigma_{\text{ee}} = 0.003\text{ m}$, $\sigma_{\text{goal}} = 0.008\text{ m}$) into indices `[6:9]` ($ee_{xyz}$) and `[9:12]` ($\Delta_{xyz}$) of `observation.state` strictly during training to improve physical robustness against servo backlash.
  - Downloads `lerobot/smolvla_base` and patches `policy_preprocessor.json`.
* **Cell 4**: Generates `train_config.yaml` and launches distributed training via `accelerate launch --multi_gpu --num_processes=2`:
  - Batch size: 8 per GPU (Effective batch size = 16).
  - Horizon: 13,000 steps.
  - Chunk size: 30 steps (1.20s action window).
  - Visual resolution: $384 \times 384$ px (wrist, side, overhead).
  - Automatically pushes final checkpoint weights and checkpoints to Hugging Face Hub at `DuyKhongCay/lekiwi_chess_smolvla`.

---

## ⚙️ Key Hyperparameter Reference

```yaml
dataset:
  repo_id: "DuyKhongCay/lekiwi_chess_v7"
  video_backend: "pyav"
policy:
  path: "/kaggle/working/smolvla_base_patched"
  repo_id: "DuyKhongCay/lekiwi_chess_smolvla"
  push_to_hub: true
  chunk_size: 30
  n_action_steps: 30
  resize_imgs_with_padding: [384, 384]
  freeze_vision_encoder: true
  train_expert_only: true
  train_state_proj: true
  optimizer_lr: 1.0e-4
  scheduler_decay_steps: 35000
steps: 13000
batch_size: 8   # per device (effective batch size: 16)
num_workers: 2
seed: 1000
```

