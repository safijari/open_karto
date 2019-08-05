# Open Karto (Python)

This is an effort to make a python package for open_karto that is pip installable.

For now, some external dependencies may need to come from apt, these include:
- libboost-all
- libeigen3

With the added wheel building step, they may not be needed when installing from PyPI

# Exposed parameters and default values
```
| angle_variance_penalty | 0.3490658503988659 |
| coarse_angle_resolution | 0.03490658503988659 |
| coarse_search_angle_offset | 0.3490658503988659 |
| correlation_search_space_dimension | 0.3 |
| correlation_search_space_resolution | 0.01 |
| correlation_search_space_smear_deviation | 0.03 |
| distance_variance_penalty | 0.3 |
| do_loop_closing | True |
| fine_search_angle_offset | 0.003490658503988659 |
| link_match_minimum_response_fine | 0.8 |
| link_scan_maximum_distance | 10.0 |
| loop_match_maximum_variance_coarse | 0.4 |
| loop_match_minimum_chain_size | 10 |
| loop_match_minimum_response_coarse | 0.8 |
| loop_match_minimum_response_fine | 0.8 |
| loop_search_maximum_distance | 4.0 |
| loop_search_space_dimension | 8.0 |
| loop_search_space_resolution | 0.05 |
| loop_search_space_smear_deviation | 0.03 |
| min_travel_distance | 0.2 |
| min_travel_heading | 10.0 |
| minimum_angle_penalty | 0.9 |
| minimum_distance_penalty | 0.5 |
| minimum_time_interval | 3600.0 |
| minimum_travel_distance | 0.2 |
| minimum_travel_heading | 10.0 |
| scan_buffer_maximum_scan_distance | 20.0 |
| scan_buffer_size | 70 |
| use_response_expansion | False |
| use_scan_barycenter | True |
| use_scan_matching | True |
```
