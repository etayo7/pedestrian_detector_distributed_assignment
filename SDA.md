# Classes & Variables

## YOLO

### YOLOFiles

- **String** `path` = Path to YOLO configuration files

- **String** `weight` = Name of `.weights` file

- **String** `cfg` = Name of `.cfg` file

- **String** `names` = Name of `.names` file

### YOLONetwork

- **cv::dnn::NET** `net` = Neural network structure

- **List<String>** `classes` = Classes names from the `.names` file

- **Vector<String>** `output_layers` = Names of output layers

### YOLOParameters

- **Float** `min_score` = Minimum fitness score for YOLO results `[0..1]`
- **Float** `max_overlap` = Maximum overlap between YOLO resulting windows `[0..1]`
- **Float** `max_count` = Maximum number of windows to consider *(Used when working with variable number of targets)*

## ReID

- **List<ReIDGallery>** `galleries` = Collection of descriptor galleries, each pertaining to a different re-identifiable person
- **torchreid::FeatureExtractor** `extractor` = Feature extractor for processing images with OSNET

### ReIDFiles

- **String** `path` = Path to YOLO configuration files

- **String** `model` = Model name *(from standard list)

### ReIDParameters

- **Integer** `descriptor_size` = For preinitialized descriptors, the number of feature arrays in each gallery, otherwise the maximum number of feature arrays in each gallery
- **Integer** `window_w` = OSNET window chosen width
- **Float** `window_ratio` = OSNET window aspect ratio *(default should be set at 2, so `H=2*W`)*

### ReIDGallery

- **List<Array>** `descriptors` = List of descriptor arrays for one re-Identifiable person

## Window

- **Float** `x` = Center X coordinate
- **Float** `y` = Center Y coordinate
- **Float** `h` = Total height of window
- **Float** `w` = Total width of window

## ProcessWindow

- **Window** `yolo_window` = YOLO window coordinates
- **Window** `osnet_window` = OSNET window coordinates
- **Integer** `assignement` = Index of descriptor gallery to which the window is assigned

## Detection parameters

**Integer** `total_targets` = Total number of targets
