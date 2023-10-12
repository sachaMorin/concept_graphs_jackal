import os

import json
from typing import Any
import numpy as np
import openai
from matplotlib import pyplot as plt

openai.api_key = os.getenv("OPENAI_API_KEY")

try: 
    # from groundingdino.util.inference import Model
    from segment_anything import sam_model_registry, SamPredictor, SamAutomaticMaskGenerator
except ImportError as e:
    print("Import Error: Please install Grounded Segment Anything following the instructions in README.")
    raise e

if "GSA_PATH" in os.environ:
    GSA_PATH = os.environ["GSA_PATH"]
    
    # Segment-Anything checkpoint
SAM_ENCODER_VERSION = "vit_h"
SAM_CHECKPOINT_PATH = os.path.join(GSA_PATH, "./sam_vit_h_4b8939.pth")

def read_json_file(filepath):
    # Read the uploaded JSON file
    content = None
    with open(filepath, "r") as f:
        content = f.read()
    data = json.loads(content)
    return data


def query_llm_old(query, system_prompt, scene_json):
    chat_completion = openai.ChatCompletion.create(
        model="gpt-4-0613",
        messages=[
            {
                "role": "system",
                "content": system_prompt,
            },
            {
                "role": "user",
                "content": json.dumps(scene_json),
            },
            {"role": "assistant", "content": "I'm ready."},
            {
                "role": "user",
                "content": query,
            },
        ],
        temperature=0,
        max_tokens=512,
        top_p=0,
        frequency_penalty=0,
        presence_penalty=0,
    )
    response = None
    try:
        # Try parsing the response as JSON
        response = json.loads(chat_completion["choices"][0]["message"]["content"])
        # response = json.dumps(response, indent=4)
    except:
        # Otherwise, just print the response
        response = chat_completion["choices"][0]["message"]["content"]
        print("NOTE: The language model did not produce a valid JSON")

    return response

def find_objects_by_ids(object_list, target_ids):
    return [obj for obj in object_list if obj['id'] in target_ids]


def query_llm(query, system_prompt, scene_desc):
    CHUNK_SIZE = 80  # Adjust this size as needed

    scene_desc_chunks = [scene_desc[i:i + CHUNK_SIZE] for i in range(0, len(scene_desc), CHUNK_SIZE)]
    aggregate_relevant_objects = []

    # for idx, chunk in enumerate(scene_desc_chunks):

    for idx in range(len(scene_desc_chunks) + 1):

        if idx < len(scene_desc_chunks):
            chunk = scene_desc_chunks[idx]
        else:  # On last iteration pass the aggregate_relevant_objects
            print(f"final query")
            chunk = aggregate_relevant_objects
            print(f"chunk : {chunk}")

        scene_desc_str = json.dumps(chunk)
        num_tokens = len(scene_desc_str.split())

        chat_completion = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": json.dumps(chunk)},
                {"role": "assistant", "content": "I'm ready."},
                {"role": "user", "content": query},
            ],
            temperature=1,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0,
        )

        response = json.loads(chat_completion["choices"][0]["message"]["content"])
        print(f"Line 183, response : {response}")

        curr_relevant_objects = find_objects_by_ids(
            chunk, response['final_relevant_objects'])

        aggregate_relevant_objects.extend(curr_relevant_objects)
    try:
        # Try parsing the response as JSON
        response = json.loads(chat_completion["choices"][0]["message"]["content"])
        # response = json.dumps(response, indent=4)
    except:
        # Otherwise, just print the response
        response = chat_completion["choices"][0]["message"]["content"]
        print("NOTE: The language model did not produce a valid JSON")

    return response


def get_sam_predictor(variant: str, device: str) -> SamPredictor:
    if variant == "sam":
        sam = sam_model_registry["vit_h"](checkpoint=SAM_CHECKPOINT_PATH)
        sam.to(device)
        sam_predictor = SamPredictor(sam)
        return sam_predictor
    
    
def get_sam_mask_generator(variant:str, device: str) -> SamAutomaticMaskGenerator:
    if variant == "sam":
        sam = sam_model_registry["vit_h"](checkpoint=SAM_CHECKPOINT_PATH)
        sam.to(device)
        mask_generator = SamAutomaticMaskGenerator(
            model=sam,
            points_per_side=12,
            points_per_batch=144,
            pred_iou_thresh=0.88,
            stability_score_thresh=0.95,
            crop_n_layers=0,
            min_mask_region_area=100,
        )
        return mask_generator
    elif variant == "fastsam":
        raise NotImplementedError
    else:
        raise NotImplementedError

# The SAM based on automatic mask generation, without bbox prompting
def get_sam_segmentation_dense(
    variant:str, model: Any, image: np.ndarray
) -> tuple:
    '''
    The SAM based on automatic mask generation, without bbox prompting
    
    Args:
        model: The mask generator or the YOLO model
        image: )H, W, 3), in RGB color space, in range [0, 255]
        
    Returns:
        mask: (N, H, W)
        xyxy: (N, 4)
        conf: (N,)
    '''
    if variant == "sam":
        results = model.generate(image)
        mask = []
        xyxy = []
        conf = []
        for r in results:
            mask.append(r["segmentation"])
            r_xyxy = r["bbox"].copy()
            # Convert from xyhw format to xyxy format
            r_xyxy[2] += r_xyxy[0]
            r_xyxy[3] += r_xyxy[1]
            xyxy.append(r_xyxy)
            conf.append(r["predicted_iou"])
        mask = np.array(mask)
        xyxy = np.array(xyxy)
        conf = np.array(conf)
        return mask, xyxy, conf
    elif variant == "fastsam":
        # The arguments are directly copied from the GSA repo
        results = model(
            image,
            imgsz=1024,
            device="cuda",
            retina_masks=True,
            iou=0.9,
            conf=0.4,
            max_det=100,
        )
        raise NotImplementedError
    else:
        raise NotImplementedError


def plot_first_five_masks_and_boxes(image, masks, xyxy, conf):
    """
    Plots up to the first 5 masks and bounding boxes on the original image.

    Parameters:
    -----------
    image: numpy.ndarray
        The original image.
    masks: list of numpy.ndarray
        List of boolean masks for object instances.
    xyxy: list of tuples
        List of bounding boxes, each defined by (x_min, y_min, x_max, y_max).
    conf: list of float
        Confidence scores associated with each mask and bounding box.

    Notes:
    -----
    This function uses matplotlib to generate a 1xn subplot, where each subplot shows one of the first 5 masks and bounding boxes on the original image, along with the associated confidence score.
    """
    n = min(5, len(masks))  # Plot up to the first 5 masks
    fig, axarr = plt.subplots(1, n, figsize=(15, 5), squeeze=False)

    for i in range(n):
        ax = axarr[0][i]
        ax.imshow(image)

        # Overlay the mask
        mask = masks[i]
        green_mask = np.zeros((*mask.shape, 3), dtype=np.uint8)
        green_mask[mask] = [0, 255, 0]  # Green color where mask is True
        ax.imshow(green_mask, alpha=0.3)

        # Draw the bounding box
        x1, y1, x2, y2 = xyxy[i]
        rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)

        # Add confidence level as title
        ax.set_title(f"Confidence: {conf[i]:.2f}")
        ax.axis('off')

    plt.tight_layout()
    plt.show()


def crop_and_pad_image(image, x_min, y_min, x_max, y_max, padding=0):
    """
    Crops and optionally pads an image based on the given bounding box dimensions.

    Parameters:
    -----------
    image: numpy.ndarray
        The original image.
    x_min, y_min, x_max, y_max: int
        Coordinates defining the bounding box to crop.
    padding: int, optional
        Number of pixels to pad around the cropped image.

    Returns:
    --------
    cropped_image: numpy.ndarray
        The cropped and optionally padded image.
    padding_values: tuple
        Tuple containing the actual padding values for left, top, right, and bottom edges of the image.

    Notes:
    -----
    1. Padding is applied within the limits of the original image dimensions.
    2. The bounding box coordinates can be adjusted based on the applied padding.
    """
    image_height, image_width = image.shape[:2]

    left_padding = min(padding, x_min)
    top_padding = min(padding, y_min)
    right_padding = min(padding, image_width - x_max)
    bottom_padding = min(padding, image_height - y_max)

    x_min -= left_padding
    y_min -= top_padding
    x_max += right_padding
    y_max += bottom_padding

    cropped_image = image[y_min:y_max, x_min:x_max]

    return cropped_image, (left_padding, top_padding, right_padding, bottom_padding)


def plot_top_5_similar_crops(similarities):
    """
    Plots the top 5 image crops based on their cosine similarities.

    Parameters:
    -----------
    similarities: list of tuples
        Each tuple contains the cosine similarity score and the corresponding cropped image.

    Notes:
    -----
    This function uses matplotlib to generate a 1x5 subplot, where each subplot shows one of the top 5 similar image crops.
    """
    fig, axarr = plt.subplots(1, 5, figsize=(20, 4))
    for i, (sim, img) in enumerate(similarities):
        ax = axarr[i]
        ax.imshow(np.array(img))
        ax.set_title(f"Cosine Sim: {sim:.4f}")
        ax.axis('off')
    plt.show()


def plot_llava_decisions(image_crops, llava_decisions):
    """
    Plots image crops with LLaVA decisions as titles.

    Parameters:
    -----------
    image_crops: list
        List of cropped images.
    llava_decisions: list
        List of decisions made by LLaVA, such as "Yes" or "No".

    Notes:
    -----
    This function uses matplotlib to create subplots where each subplot shows an image crop along with the LLaVA decision as its title.
    """
    fig, axes = plt.subplots(1, len(image_crops), figsize=(20, 5))

    for ax, crop, decision in zip(axes, image_crops, llava_decisions):
        ax.imshow(np.array(crop))
        ax.set_title(f"LLaVA: {decision}")
        ax.axis("off")

    plt.show()


def get_valid_crops(image, xyxy, mask, padding=0):
    """
    Extracts valid image crops and their corresponding masks and indices based on bounding box dimensions.

    Parameters:
    -----------
    image: numpy.ndarray
        The original image.
    xyxy: list of tuples
        List of bounding boxes, each defined by (x_min, y_min, x_max, y_max).
    mask: numpy.ndarray or tensor
        Segmentation mask for the objects in the image.
    padding: int, optional
        Number of pixels to pad around each bounding box.

    Returns:
    --------
    valid_crops: list
        List of valid cropped images.
    valid_indices: list
        Indices of the bounding boxes that were used to generate the valid crops.
    valid_masks: list
        Cropped masks corresponding to the valid crops.

    Notes:
    -----
    1. A bounding box is considered valid if both its width and height are greater than 20 pixels.
    2. The cropping takes into account the specified padding.
    3. Mask shape is assumed to be (N, height, width), where N is the number of instances.
    """
    valid_crops = []
    valid_indices = []
    valid_masks = []

    for i, (x_min, y_min, x_max, y_max) in enumerate(xyxy):
        if not (x_max - x_min > 20 and y_max - y_min > 20):
            continue

        cropped_image, _ = crop_and_pad_image(image, x_min, y_min, x_max, y_max, padding)
        valid_crops.append(cropped_image)

        # Make sure you're getting the correct slice for the mask, assuming mask shape is (N, height, width)
        cropped_mask, _ = crop_and_pad_image(mask[i], x_min, y_min, x_max, y_max, padding)
        valid_masks.append(cropped_mask)

        valid_indices.append(i)

    return valid_crops, valid_indices, valid_masks
