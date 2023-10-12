#!/home/sacha/CFSLAM_/venv/bin/python3
import os

import torch
import argparse
import rospy
from real_nav.srv import QueryService, DoISee
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from llava_model import LLaVaChat
from PIL import Image
import cv2

from utils import get_sam_segmentation_dense, read_json_file, query_llm, get_sam_mask_generator, get_valid_crops

import numpy as np

import open_clip

try:
    # from groundingdino.util.inference import Model
    from segment_anything import sam_model_registry, SamPredictor, SamAutomaticMaskGenerator
except ImportError as e:
    print("Import Error: Please install Grounded Segment Anything following the instructions in README.")
    raise e


class ConceptGraphTools:
    def __init__(self, system_prompt_path, scene_json_path, object_detector):
        super().__init__()
        # Set this to false if you use the client in another ros node
        # Initialize Node
        rospy.init_node("ConceptGraphTools", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Load System Prompt and JSON scene description
        self.system_prompt = open(system_prompt_path, "r").read()
        self.scene_desc = read_json_file(scene_json_path)

        # Filter out some objects
        n_objects = len(self.scene_desc)
        self.scene_desc = [o for o in self.scene_desc if o["object_tag"] not in ["invalid", "none", "unknown"]]
        new_n_objects = len(self.scene_desc)
        rospy.loginfo(
            f"Removed {n_objects - new_n_objects} objects from the scene with invalid tags. {new_n_objects} left.")

        # Remove possible_tags
        for o in self.scene_desc:
            # o.pop("possible_tags")
            o.pop("bbox_extent")

        # Publishers
        self.location_publisher = rospy.Publisher('/cf_object_location', Float32MultiArray, queue_size=10)

        # Subscribers
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.camera_callback,
                                           queue_size=1)

        # Visited objects
        self.visited_objects = []

        # Image buffer
        height = 480
        width = 640
        self.image = np.zeros((height, width, 3), np.uint8)
        self.image_shape = (width, height)

        # open image from image_path and turn it into a numpy array
        # image_path = "/home/sacha/Downloads/image.png"
        # self.image = cv2.imread(image_path)
        # self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        if object_detector == "llava_full_image":
            # LLaVa Chat
            print("Loading LLaVA Model")
            self.llava = LLaVaChat(os.environ["LLAVA_CKPT_PATH"], "v0_mmtag", 1)
            print("Loaded LLaVA Model")
            do_i_see_cb = self.do_i_see
        elif object_detector == "sam_clip":
            # Get SAM Model
            # self.sam_predictor = get_sam_predictor("sam", "cuda")
            print("Loading SAM Model")
            self.sam_mask_generator = get_sam_mask_generator("sam", "cuda")
            print("Loaded SAM Model")

            # Initialize the CLIP model
            print("Loading CLIP Model")
            self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms(
                "ViT-H-14", "laion2b_s32b_b79k")
            self.clip_model = self.clip_model.to("cuda")
            self.clip_tokenizer = open_clip.get_tokenizer("ViT-H-14")
            print("Loaded CLIP Model")
            do_i_see_cb = self.check_image_sam_clip
        else:
            raise ValueError("Wrong object_detector value.")

        # Services
        self.query_goal = rospy.Service("concept_graph_tools/query_goal", QueryService, self.query_goal)
        self.do_i_see = rospy.Service("concept_graph_tools/do_i_see", DoISee, do_i_see_cb)

        rospy.loginfo("SERVICES READY!")

    def query_goal(self, req):
        """Process query and publish goal pose of relevant object to /cf_object_location."""
        query = req.query
        visual = req.visual
        excluded_ids = req.excluded_ids

        if len(excluded_ids):
            scene = [o for o in self.scene_desc if o["id"] not in excluded_ids]
        else:
            scene = self.scene_desc

        if visual:
            llava_query = "Describe the current image. If you see a text or a sign, read it and include the content in your answer."
            rospy.loginfo("Parsing Visual Query...")
            image_desc = self.llava(query=llava_query, image_features=self.get_image_features())
            rospy.loginfo(f"I see an image described as '{image_desc}'")
            query_gpt = f"I see an image described as '{image_desc}' and and would like you to find an object matching the query '{query}'."
            self.llava.reset()
        else:
            query_gpt = query


        self.object_desc_base = query
        # query_gpt = f"'The object described as '{query}' is not in the scene. Find a likely container or storage space where someone is likely to have moved the object described as '{query}'?"
        query_gpt = (f"The object described as '{self.object_desc_base}' is not in the scene. "
                 f"Perhaps someone has moved it, or put it away. "
                 f"Let's try to find the object by visiting the likely places, storage or containers that are "
                 f"appropriate for the missing object (eg: a a cabinet for a wineglass, or closet for a broom). "
                 f"So the new query is find a likely container or storage space where someone typically would"
                 f"have moved the object described as '{self.object_desc_base}'?")
        # query_gpt = (f"The objects in the scene were found in a house. Find an a storage space or container "
        #              f"where would you expect to find '{self.object_desc_base}' in a typical house.")

        response = query_llm(query_gpt, self.system_prompt, scene)
        rospy.loginfo("GPT Response")
        rospy.loginfo(response)

        query_achievable = response["query_achievable"]

        if query_achievable:
            object_id = response["final_relevant_objects"][0]
            object_desc = response["most_relevant_desc"]

            # Find object data
            for object_data in self.scene_desc:
                if object_data["id"] == object_id:
                    break

            location_message = Float32MultiArray(data=object_data["bbox_center"])

            # Publish the message
            self.location_publisher.publish(location_message)
            self.visited_objects.append(object_id)
        else:
            object_id, object_desc = -1, "NOT AN OBJECT"

        return dict(object_id=object_id, object_desc=object_desc, query_achievable=query_achievable)

    def extract_clip_features_and_check_similarity(self, valid_crops, object_desc, similarity_threshold=0.26):
        """
        Extracts CLIP features from a list of image crops and checks their similarity against a text description.

        Parameters:
        -----------
        valid_crops: list
            List of cropped images in numpy array format.
        object_desc: str
            Textual description of the object.
        similarity_threshold: float, optional
            Cosine similarity threshold for determining a match.

        Returns:
        --------
        valid_crops: list
            List of original valid cropped images.
        image_feats: numpy.ndarray
            Concatenated array of feature vectors of each crop.
        most_similar_crop: numpy.ndarray
            Crop with the highest similarity to the text description.
        is_similar_flag: bool
            Indicates if any crop has similarity greater than the threshold.
        sorted_sims: list
            Top 5 similarity scores and their corresponding cropped images, sorted in descending order.

        Function Flow:
        1. Tokenize and encode the object description using CLIP.
        2. Normalize the text features.
        3. Loop through each valid crop to extract and normalize its CLIP features.
        4. Compute cosine similarity between text features and each image feature.
        5. Determine if any crop has a similarity above the given threshold.
        6. Return results.

        Notes:
        -----
        The code assumes that the CLIP model and tokenizer are already loaded into self.clip_model and self.clip_tokenizer.
        """
        with torch.no_grad():
            image_feats = []
            max_similarity = -1
            most_similar_crop = None
            is_similar_flag = False

            # Tokenize the object description and get its feature
            tokenized_text = self.clip_tokenizer([object_desc]).to("cuda")
            text_feat = self.clip_model.encode_text(tokenized_text)
            text_feat /= text_feat.norm(dim=-1, keepdim=True)
            text_feat = text_feat.cpu().numpy()

            similarity_scores = []

            for cropped_image in valid_crops:
                preprocessed_image = self.clip_preprocess(Image.fromarray(cropped_image)).unsqueeze(0).to("cuda")

                # preprocessed_image = self.clip_preprocess(cropped_image).unsqueeze(0).to("cuda")
                crop_feat = self.clip_model.encode_image(preprocessed_image)
                crop_feat /= crop_feat.norm(dim=-1, keepdim=True)
                crop_feat = crop_feat.cpu().numpy()

                # Compute the cosine similarity
                sim = np.dot(text_feat, crop_feat.T).flatten()[0]
                similarity_scores.append((sim, cropped_image))

                if sim > max_similarity:
                    max_similarity = sim
                    most_similar_crop = cropped_image

                if sim > similarity_threshold:
                    is_similar_flag = True

                image_feats.append(crop_feat)

            image_feats = np.concatenate(image_feats, axis=0)

            sorted_sims = sorted(similarity_scores, key=lambda x: x[0], reverse=True)[:5]

        return valid_crops, image_feats, most_similar_crop, is_similar_flag, sorted_sims

    def check_objects_in_crops(self, valid_crops, valid_indices, valid_masks, xyxy, object_desc, black_out=False):
        """
        Checks for the presence of an object described by 'object_desc' in a list of image crops.

        Parameters:
        ----------
        valid_crops: list
            List of cropped images.
        valid_indices: list
            Indices corresponding to valid cropped images.
        valid_masks: list
            List of masks corresponding to each crop.
        xyxy: list
            List of bounding boxes in the format [(x_min, y_min, x_max, y_max), ...].
        object_desc: str
            Description of the object to find.
        black_out: bool, optional
            Whether to black out the regions outside the masks.

        Returns:
        -------
        object_found_in_any_crop: bool
            Flag indicating if the object is found in any of the valid crops.
        llava_decisions: list
            List containing the LLaVA model's decisions ("Yes"/"No") for each crop.

        Function Flow:
        1. Loop through each valid crop and its corresponding mask.
        2. Optionally black out the area outside the mask.
        3. Preprocess the image and obtain its features.
        4. Use LLaVA to check for the object's presence.
        5. Store LLaVA's decision and reset its state.

        Notes:
        -----
        Uncomment the lines at the end to stop the loop once the object is found in any crop.
        """
        object_found_in_any_crop = False
        llava_decisions = []

        for i, index in enumerate(valid_indices):
            crop = valid_crops[i]
            local_mask = valid_masks[i] if black_out else None

            if black_out:
                # Convert PIL image to numpy array and apply mask
                crop_np = np.array(crop)
                blacked_out_crop_np = np.zeros_like(crop_np)
                blacked_out_crop_np[local_mask] = crop_np[local_mask]

                # Convert back to PIL for processing
                crop = Image.fromarray(blacked_out_crop_np)

            image_tensor = self.llava.image_processor.preprocess(crop, return_tensors="pt")["pixel_values"][0]
            image_features = self.llava.encode_image(image_tensor[None, ...].half().cuda())

            query = f"I am looking for an object described as '{object_desc}'. Is it in the image?"
            outputs = self.llava(query=query, image_features=image_features)

            object_in_crop = "Yes" in outputs or "yes" in outputs
            llava_decisions.append("Yes" if object_in_crop else "No")
            self.llava.reset()

            # if object_in_crop:
            #     object_found_in_any_crop = True
            #     break  # Exit the loop if the object is found in any crop

        return object_found_in_any_crop, llava_decisions

    def check_image_sam_clip(self, req):
        """
        Checks if an object described by 'req' is present in an image.

        Parameters:
        ----------
        req: str
            Object description (query).

        Function Flow:
        1. Generates segmentation mask, bounding boxes (xyxy), and confidence scores (conf) using SAM.
        2. Creates valid image crops along with their masks based on these bounding boxes.
        3. Extracts features using CLIP and checks for similarity between object description and image crops.
        4. Optionally, can also use LLaVA for more detailed checking (commented out).

        Returns:
        -------
        is_similar_flag: bool
            Flag indicating if the object description is similar to any of the image crops.

        Notes:
        -----
        - Use 'plot_first_five_masks_and_boxes' to debug mask and bounding box extraction.
        - 'plot_top_5_similar_crops' visualizes the crops with highest similarity to the object description.
        - Uncomment 'check_objects_in_crops' and 'plot_llava_decisions' to employ LLaVA for object checking.
        """
        object_desc = req.query

        if True:  # "class_set" == "none":
            mask, xyxy, conf = get_sam_segmentation_dense(
                "sam", self.sam_mask_generator, self.image)
            # plot_first_five_masks_and_boxes(self.image, mask, xyxy, conf)

            valid_crops, valid_indices, valid_masks = get_valid_crops(self.image, xyxy, mask, padding=15)

            # Extract CLIP features and check similarity
            image_crops, image_feats, most_similar_crop, is_similar_flag, top_5_similarities = self.extract_clip_features_and_check_similarity(
                valid_crops, object_desc)

            if is_similar_flag:
                print(f"Object desc {object_desc} was found in the image.")
            else:
                print(f"Object desc {object_desc} was NOT found in the image.")
            print(f"Line 332, is_similar_flag: {is_similar_flag}")

            # plot_top_5_similar_crops(top_5_similarities)

            #  # Checking for the object in each crop using LLaVA
            # object_found, llava_decisions = self.check_objects_in_crops(valid_crops, valid_indices, valid_masks, xyxy, object_desc, black_out=False)
            # plot_llava_decisions(valid_crops, llava_decisions)

            k = 2

        k = 1
        return is_similar_flag

    def get_image_features(self):
        image = Image.fromarray(self.image)
        image_tensor = self.llava.image_processor.preprocess(image, return_tensors="pt")[
            "pixel_values"
        ][0]
        image_features = self.llava.encode_image(image_tensor[None, ...].half().cuda())
        return image_features

    def do_i_see(self, req):
        """Ask LLaVA if object is seen in camera image."""
        object_desc = req.query

        query = "List the set of objects in this image. Describe the colors, materials and other properties of each object."
        outputs = self.llava(query=query, image_features=self.get_image_features())
        rospy.loginfo(f"User Query : {query}")
        rospy.loginfo(f"LLAVA      : {outputs}")

        query = f"I am looking for an object described as '{object_desc}'. Is it in the image?"
        outputs = self.llava(query=query, image_features=None)
        rospy.loginfo(f"User Query : {query}")
        rospy.loginfo(f"LLAVA      : {outputs}")

        query = f"Given your previous answer, now answer 'Yes' or 'No'. Answer with a single word."
        outputs = self.llava(query=query, image_features=None)
        rospy.loginfo(f"User Query : {query}")
        rospy.loginfo(f"LLAVA      : {outputs}")

        object_in_image = "Yes" in outputs or "yes" in outputs

        self.llava.reset()

        return object_in_image

    def camera_callback(self, data):
        bridge = CvBridge()

        try:
            image = bridge.compressed_imgmsg_to_cv2(data)
            self.image = image[:, :, ::-1]
        except CvBridgeError as e:
            print(e)

    def spin(self):
        rospy.spin()

    def on_shutdown(self):
        pass


if __name__ == '__main__':
    # Argparse with a system_prompt_path and a scene_json_path argument
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--system_prompt_path", type=str, default="prompts/concept_graphs_planner.txt")
    parser.add_argument("-s", "--scene_json_path", type=str, default="scene.json")
    # add argument for sim threshold
    parser.add_argument("-t", "--sim_threshold", type=float, default=0.26)
    parser.add_argument("-o", "--object_detector", type=str, default="llava_full_image")
    args = parser.parse_args()

    node = ConceptGraphTools(args.system_prompt_path, args.scene_json_path, args.object_detector)
    node.spin()
