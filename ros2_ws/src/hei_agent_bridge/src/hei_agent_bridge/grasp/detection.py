"""Explicit scene vocabulary recovery for front RGB-D; no category inference."""
from .compute_device import inference_device
import cv2
import numpy as np

from .common_utils import grasp_target
from .obb import segmented_cloud


def prompt_aliases(value):
    if type(value) is not dict or len(value) > 64:
        raise ValueError('detector_prompt_aliases must be an object with at most 64 targets')
    result = {}
    for target, aliases in value.items():
        name = grasp_target(target)
        if type(aliases) is not list or len(aliases) > 3:
            raise ValueError('Each target allows at most three explicit detector descriptions')
        if name in result:
            raise ValueError('Duplicate normalized detector target')
        result[name] = list(dict.fromkeys(grasp_target(alias) for alias in aliases))
    return result


def detect_cloud(detector, image, depth, intrinsic, target, transform, aliases, diagnostics, directory):
    """Retry descriptions only after zero matching detections, never a geometry failure."""
    target = grasp_target(target)
    prompts = list(dict.fromkeys([target, *prompt_aliases(aliases).get(target, [])]))
    attempts = []
    diagnostics.update(attempts=attempts, requested_target=target,compute_device=inference_device())
    for prompt in prompts:
        detector.set_classes([prompt])
        results = detector.predict(image, verbose=False, device=inference_device(), conf=.25, iou=.45,
                                   imgsz=1280, retina_masks=True)
        report = dict(prompt=prompt, confidence_threshold=.25, inference_size=1280)
        attempts.append(report)
        try:
            score, points, mask = segmented_cloud(results, depth, intrinsic, prompt, transform, report)
        except ValueError as error:
            report.update(status='rejected', error=str(error))
            diagnostics.update(report)
            diagnostics['target'] = target
            if report.get('matching_detections', 0) or prompt == prompts[-1]:
                raise
            continue
        report['status'] = 'selected'
        diagnostics.update(report)
        diagnostics.update(target=target, selected_prompt=prompt)
        diagnostics.pop('error', None)
        cv2.imwrite(str(directory / 'perception_mask.png'), mask.astype(np.uint8) * 255)
        overlay = image.copy()
        overlay[mask] = (overlay[mask] * .5 + np.array([0, 180, 255]) * .5).astype(np.uint8)
        cv2.imwrite(str(directory / 'perception_overlay.jpg'), overlay)
        return score, points, mask, prompt
    raise AssertionError('At least one detector prompt is required')
