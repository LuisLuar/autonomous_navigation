import torch
import torch.nn as nn
from transformers import SegformerForSemanticSegmentation

class SegFormer(nn.Module):
    def __init__(self, num_classes, checkpoint="nvidia/segformer-b5-finetuned-cityscapes-1024-1024"):
        super().__init__()
        self.model = SegformerForSemanticSegmentation.from_pretrained(
            checkpoint,
            num_labels=num_classes,
            ignore_mismatched_sizes=True
        )

    def forward(self, x):
        out = self.model(x)
        return out.logits
