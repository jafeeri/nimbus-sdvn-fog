"""paper_model.py - EVERY equation and parameter from the professor's document

Source document: "STMM OG PAPER UPDATED" (equations and values supplied by
Dr. Huma Ghafoor), which consolidates:

  [STMM]     Khanam, Basharat, Ghafoor, Koo, "Safe Through mmWave in Mist
             (STMM)", IEEE Sensors Journal 2025 - Eqs. (1), (5), (6), (7)
             and Table II simulation parameters.
  [NYUSIM]   The close-in (CI) path-loss model with atmospheric attenuation,
             prescribed in the document for V2V mmWave links in fog
             (document Eqs. (1)-(5) of the path-loss section).
  [UMI]      3GPP TR 38.901 Urban-Micro LOS path loss, prescribed in the
             document for V2I links (RSU modelled as the 3GPP Base Station,
             vehicle as the User Equipment).
  [GIORDANI] "Path Loss Models for V2V mmWave Communication" Table IV values,
             reproduced in the document as "Values Used Throughout the Paper"
             (63 GHz, 1 GHz bandwidth, 21 dBm, NF 13 dB, 3 lanes x 4 m,
             Type-2/Type-3 vehicles with 1.6 m / 3 m antenna heights).

This module is deliberately self-contained and written with FULLY DESCRIPTIVE
variable names so a reader can hold the document in one hand and this file in
the other and match them line by line. The rest of the simulator imports its
physics from here - these functions are load-bearing, not decoration.

--------------------------------------------------------------------------
PARAMETERS PRESCRIBED BY THE DOCUMENT (symbol -> descriptive name -> value)
--------------------------------------------------------------------------
r_max        maximum_communication_range_m      70 m mmWave V2V,
                                                200 m DSRC V2V,
                                                500 m RSU/LC
v_i, v_j     vehicle_speed_ms                    10-15 m/s (dynamic)
vis_c        current_visibility_m                5-20 m (dynamic with fog)
vis_max      maximum_visibility_m                50 m
d_ij         inter_vehicle_distance_m            from vehicle coordinates
(x_i, y_i)   vehicle coordinates                 from mobility simulation
theta        angle_toward_destination_rad        arctan((y_Des-y_i)/(x_Des-x_i))
-            number_of_vehicles                  10-100
-            vehicle_spacing_safety_m            20-70 m (70 m chosen for
                                                mmWave evaluation)
-            drone_height_m                      50 m
-            drone_radio_technology              mmWave
-            drone_trajectory                    hovering over highway
-            drones_per_km                       2
Table II     simulation_area                     1500 m x 100 m
Table II     data_rate                           10 Gb/s mmWave, 10 Mb/s DSRC
-            packet_size                         200 B (Dr. Ghafoor: both techs)
Table II     carrier_frequency                   65 GHz mmWave (Table II) /
                                                63 GHz (document p.6,
                                                "values used throughout");
                                                we adopt 63 GHz - see note
Table II     fading_model                        Rician (mmWave; per
                                                Dr. Ghafoor, K = 8 dB,
                                                refining the Table II
                                                Rayleigh entry),
                                                Nakagami m = 3 (DSRC)
Table II     transmit_power                      10 dBm (Table II) / 21 dBm
                                                (document p.6); we adopt
                                                21 dBm - see note
p.6          bandwidth                           1 GHz
p.6          noise_figure                        13 dB
p.6          highway_lanes                       3 per direction, 4 m wide
p.6          vehicle/antenna heights             1.6 m (Type 2), 3 m (Type 3)

NOTE (frequency / power): the document contains both the STMM Table II pair
(65 GHz, 10 dBm) and the Giordani "values used throughout the paper" pair
(63 GHz, 21 dBm, NF 13 dB, 1 GHz). We adopt 63 GHz / 21 dBm / NF 13 because
that is the exact configuration under which the prescribed V2V path-loss
behaviour was validated; 63 vs 65 GHz differs by 0.27 dB in FSPL. Both are
one-line changes in config.py, and this discrepancy is flagged to the
professor in the accompanying notes.
"""

from __future__ import annotations

import math

SPEED_OF_LIGHT_M_PER_S = 299_792_458.0


# ==========================================================================
# [STMM] Eq. (1) - control duration D of a Local Controller seen by the MC
# ==========================================================================
def control_duration_D_seconds(
    distance_mc_to_lc_m: float,
    balancing_constant_alpha: float,
    packet_size_bits: float,
    backhaul_link_capacity_bps: float,
    hello_window_t_s: float,
    number_of_hello_messages: float,
    controller_hop_count: int,
) -> float:
    """[STMM] Eq. (1):

        D = ( d_MC,j / (alpha * speed)  +  P_s / C_MC,j  +  t / hellomessages )
            * HopCount

    where `speed` is the speed of light on the I2I backhaul, P_s the packet
    size, C_MC,j the backhaul capacity, and hellomessages the count of hello
    (control) messages the LC received from vehicles in t seconds.
    """
    propagation_term_s = distance_mc_to_lc_m / (balancing_constant_alpha
                                                * SPEED_OF_LIGHT_M_PER_S)
    transmission_term_s = packet_size_bits / backhaul_link_capacity_bps
    hello_term_s = hello_window_t_s / max(1.0, number_of_hello_messages)
    return (propagation_term_s + transmission_term_s + hello_term_s) \
        * controller_hop_count


# ==========================================================================
# [STMM] Eq. (5) - Visibility Time of a wireless link (with its two helper
# definitions: the Euclidean distance d_ij and the destination angle theta)
# ==========================================================================
def inter_node_distance_m(x_i: float, y_i: float, x_j: float, y_j: float) -> float:
    """[STMM] Eq. (5), middle line:  d_ij = sqrt((x_i-x_j)^2 + (y_i-y_j)^2)."""
    return math.hypot(x_i - x_j, y_i - y_j)


def angle_toward_destination_rad(x_node: float, y_node: float,
                                 x_destination: float, y_destination: float) -> float:
    """[STMM] Eq. (5), last line:  theta = arctan((y_Des - y_i)/(x_Des - x_i)).

    Implemented with atan2 so the quadrant is correct when the destination
    is behind the node (x_Des < x_i), which plain arctan cannot distinguish.
    """
    return math.atan2(y_destination - y_node, x_destination - x_node)


def visibility_time_VT_seconds(
    maximum_communication_range_m: float,
    inter_vehicle_distance_m: float,
    speed_node_i_ms: float,
    speed_node_j_ms: float,
    angle_node_i_toward_destination_rad: float,
    angle_node_j_toward_destination_rad: float,
    current_visibility_m: float,
    maximum_visibility_m: float,
) -> float:
    """[STMM] Eq. (5):

                    r_max - d_ij                              vis_c
        VT = ------------------------------------------- x ---------
             sqrt( (v_i cos th_i - v_j cos th_j)^2            vis_max
                 + (v_i sin th_i - v_j sin th_j)^2 )

    The document explains VT as: "instead of simply choosing the closest
    relay, choose the relay whose wireless link will remain connected the
    longest". The '-' branch of (r_max +/- d_ij) is used: remaining overlap
    of the coverage ranges. The visibility ratio vis_c/vis_max shortens all
    link lifetimes in fog.
    """
    relative_speed_ms = math.sqrt(
        (speed_node_i_ms * math.cos(angle_node_i_toward_destination_rad)
         - speed_node_j_ms * math.cos(angle_node_j_toward_destination_rad)) ** 2
        + (speed_node_i_ms * math.sin(angle_node_i_toward_destination_rad)
           - speed_node_j_ms * math.sin(angle_node_j_toward_destination_rad)) ** 2
    )
    relative_speed_ms = max(0.05, relative_speed_ms)  # co-moving fog convoy
    remaining_range_m = max(0.0, maximum_communication_range_m
                            - inter_vehicle_distance_m)
    visibility_ratio = min(1.0, current_visibility_m / max(1e-9, maximum_visibility_m))
    return remaining_range_m / relative_speed_ms * visibility_ratio


# ==========================================================================
# [STMM] Eq. (7) and Eq. (6) - path time and optimal path
# ==========================================================================
def path_time_PTP_seconds(visibility_times_along_path_s: list[float]) -> float:
    """[STMM] Eq. (7):  PT_P = min(VT_1,p , VT_2,p , ... , VT_n,p).

    A path is only as stable as its weakest (shortest-lived) link.
    """
    return min(visibility_times_along_path_s) if visibility_times_along_path_s else 0.0


def optimal_path_OP_seconds(path_times_of_all_candidate_paths_s: list[float]) -> float:
    """[STMM] Eq. (6):  OP = max(PT_P) over all paths p = 1..P.

    The OC provides a stable path by maximising the path time over all
    candidate paths between source and destination.
    """
    return max(path_times_of_all_candidate_paths_s) \
        if path_times_of_all_candidate_paths_s else 0.0


# ==========================================================================
# [NYUSIM] V2V mmWave path loss for the foggy environment
# (document path-loss section, Eqs. (1)-(5))
# ==========================================================================
def free_space_path_loss_FSPL_db(carrier_frequency_ghz: float,
                                 distance_m: float) -> float:
    """[NYUSIM] Eq. (2):  FSPL[dB] = 32.4 + 20 log10(f) + 20 log10(d)

    with f in GHz and d in metres. Evaluated at d = d0 = 1 m this is the
    close-in free-space reference of the CI model.
    """
    return (32.4 + 20.0 * math.log10(carrier_frequency_ghz)
            + 20.0 * math.log10(max(1e-3, distance_m)))


def atmospheric_attenuation_AT_db(attenuation_factor_alpha_db_per_m: float,
                                  distance_m: float) -> float:
    """[NYUSIM] Eq. (3):  AT[dB] = alpha[dB/m] x d[m].

    alpha is the attenuation factor for 1-100 GHz caused by atmospheric
    gases, fog, rain, snow and haze. In this simulator alpha is computed
    physically as (oxygen absorption + ITU-R P.840 fog attenuation from the
    current visibility) - see channel.py.
    """
    return attenuation_factor_alpha_db_per_m * max(0.0, distance_m)


def nyusim_close_in_path_loss_db(
    carrier_frequency_ghz: float,
    distance_m: float,
    path_loss_exponent_n: float,
    attenuation_factor_alpha_db_per_m: float,
    shadow_fading_chi_sigma_db: float,
    close_in_reference_distance_d0_m: float = 1.0,
) -> float:
    """[NYUSIM] Eq. (1) - the close-in (CI) path-loss model with weather:

        PL(f,d)[dB] = FSPL(f, d0)[dB] + 10 n log10(d/d0) + AT[dB] + chi_sigma^CI

    where n is the path-loss exponent (PLE), d0 = 1 m, chi_sigma^CI a
    zero-mean Gaussian (in dB) modelling large-scale shadowing, and AT the
    atmospheric attenuation of Eq. (3). The document prescribes this model
    for V2V mmWave links in the foggy environment.
    """
    distance_m = max(close_in_reference_distance_d0_m, distance_m)
    return (free_space_path_loss_FSPL_db(carrier_frequency_ghz,
                                         close_in_reference_distance_d0_m)
            + 10.0 * path_loss_exponent_n
            * math.log10(distance_m / close_in_reference_distance_d0_m)
            + atmospheric_attenuation_AT_db(attenuation_factor_alpha_db_per_m,
                                            distance_m)
            + shadow_fading_chi_sigma_db)


def path_loss_exponent_from_measurement(
    average_received_signal_Rs_db: float,
    attenuation_factor_alpha_db_per_m: float,
    distance_m: float,
    shadow_fading_chi_sigma_db: float,
) -> float:
    """[NYUSIM] Eq. (4) - the measurement-side identity for the PLE:

        n = ( R_s - alpha[dB/m] x d[m] - chi_sigma^CI ) / (10 log10(d))

    This is how NYUSIM extracts n from measured received signal strength.
    A forward simulation CHOOSES n (we use n = 2.0 for line-of-sight, per
    the CI model's free-space-like V2V LOS behaviour, and n = 3.0 for
    terrain-obstructed NLOS); this function is provided so every numbered
    equation of the document exists in code and can be used to sanity-check
    the implementation (see tests).
    """
    return ((average_received_signal_Rs_db
             - attenuation_factor_alpha_db_per_m * distance_m
             - shadow_fading_chi_sigma_db)
            / (10.0 * math.log10(max(1.001, distance_m))))


def shadow_fading_residual_chi_db(
    measured_path_loss_db: float,
    carrier_frequency_ghz: float,
    distance_m: float,
    path_loss_exponent_n: float,
    attenuation_factor_alpha_db_per_m: float,
) -> float:
    """[NYUSIM] Eq. (5) - the shadowing residual identity:

        chi_sigma^CI = PL^CI(f,d)[dB] - FSPL(f,d0)[dB]
                       - 10 n log10(d) - AT[dB]

    i.e. whatever the measured loss cannot be explained by the deterministic
    CI terms is attributed to large-scale shadowing. Provided for
    completeness/verification (inverse of nyusim_close_in_path_loss_db).
    """
    return (measured_path_loss_db
            - free_space_path_loss_FSPL_db(carrier_frequency_ghz, 1.0)
            - 10.0 * path_loss_exponent_n * math.log10(max(1.001, distance_m))
            - atmospheric_attenuation_AT_db(attenuation_factor_alpha_db_per_m,
                                            distance_m))


# ==========================================================================
# [UMI] V2I path loss - 3GPP TR 38.901 Urban-Micro LOS
# (document: "For the V2I link ... the RSU is modeled as the 3GPP Base
#  Station (BS), while the vehicle is modeled as the User Equipment (UE).")
# ==========================================================================
def umi_breakpoint_distance_m(carrier_frequency_ghz: float,
                              base_station_height_m: float,
                              user_equipment_height_m: float,
                              effective_environment_height_m: float = 1.0) -> float:
    """3GPP TR 38.901 breakpoint distance for the UMi-LOS dual-slope model:

        d'_BP = 4 * h'_BS * h'_UE * f_c / c,   h' = h - h_E (h_E = 1 m)
    """
    h_bs_eff = max(0.1, base_station_height_m - effective_environment_height_m)
    h_ue_eff = max(0.1, user_equipment_height_m - effective_environment_height_m)
    return (4.0 * h_bs_eff * h_ue_eff * carrier_frequency_ghz * 1e9
            / SPEED_OF_LIGHT_M_PER_S)


def umi_los_path_loss_db(distance_3d_m: float,
                         distance_2d_m: float,
                         carrier_frequency_ghz: float,
                         base_station_height_m: float,
                         user_equipment_height_m: float) -> float:
    """[UMI] document Eq. (3) - 3GPP TR 38.901 UMi-LOS path loss:

        PL_UMi-LOS =
          32.4 + 21 log10(d_3D) + 20 log10(f_c)            10 m <= d_2D <= d'_BP
          32.4 + 40 log10(d_3D) + 20 log10(f_c)
             - 9.5 log10( (d'_BP)^2 + (h_BS - h_UE)^2 )     d'_BP <= d_2D <= 5 km

    In our deployment the RSU mast and the 60-GHz-band carrier put every
    link far below the breakpoint (d'_BP is several km), so the first branch
    applies; the second branch is implemented for completeness.
    """
    d3 = max(1.0, distance_3d_m)
    breakpoint_m = umi_breakpoint_distance_m(carrier_frequency_ghz,
                                             base_station_height_m,
                                             user_equipment_height_m)
    if distance_2d_m <= breakpoint_m:
        return 32.4 + 21.0 * math.log10(d3) + 20.0 * math.log10(carrier_frequency_ghz)
    return (32.4 + 40.0 * math.log10(d3) + 20.0 * math.log10(carrier_frequency_ghz)
            - 9.5 * math.log10(breakpoint_m ** 2
                               + (base_station_height_m - user_equipment_height_m) ** 2))
