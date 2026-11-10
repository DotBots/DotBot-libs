/**
 * @file
 * @brief  Frozen copy of the LH2 decode path at DotBot-libs 228ce9a
 *
 * The function bodies below are the 228ce9a text. What is not: the names the
 * copy needs from bsp/lh2.h, bsp/lh2_decoder.h and drv/lh2_geometry.h,
 * restated here so the copy builds without them; internal linkage, given by
 * the static prototypes, so it links beside the module under test; the static
 * checkpoint table kept as its 2,048 LFSR indices, the state at each index
 * computed at init; and the hooks, at the end of the file.
 *
 * @copyright Inria, 2026
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lh2_reference.h"

//=========================== from the headers at 228ce9a ======================

#define LH2_BASESTATION_COUNT 16                         ///< Number of supported concurrent basestations
#define LH2_POLYNOMIAL_COUNT  LH2_BASESTATION_COUNT * 2  ///< Number of supported LFSR polynomials, two per basestation
#define LH2_SWEEP_COUNT       2                          ///< Number of laser sweeps per basestations rotation

#define LH2_POLYNOMIAL_ERROR_INDICATOR  0xFF        ///< indicate the polynomial index is invalid
#define LH2_LFSR_SEARCH_ERROR_INDICATOR 0xFFFFFFFF  ///< indicate the polynomial index is invalid
// Un-comment the following line if you want to enable the Anti-Mocap fiter
#define LH2_MOCAP_FILTER 1  ///< Defined when the LH2 needs to coexits with a Qualysis Mocap system. It enables harsher anti-outlier filters

#define NUM_LSFR_COUNT_CHECKPOINTS 64  ///< How many lsfr checkpoints are per polynomial

#define DB_LH2_GEOMETRY_STATIONS      (16U)
#define DB_LH2_PERIOD_TICKS_PER_COUNT (8U)

#define DB_LH2_NO_NEW_DATA              LH2_REF_NO_NEW_DATA
#define DB_LH2_PROCESSED_DATA_AVAILABLE LH2_REF_PROCESSED_DATA_AVAILABLE

typedef lh2_ref_t            db_lh2_t;
typedef lh2_ref_checkpoint_t _lfsr_checkpoint_t;

_Static_assert(LH2_POLYNOMIAL_COUNT == LH2_REF_POLYNOMIALS, "one polynomial count");
_Static_assert(NUM_LSFR_COUNT_CHECKPOINTS == LH2_REF_CHECKPOINTS, "one checkpoint count");

//=========================== prototypes =======================================

static uint64_t _demodulate_light(uint8_t *sample_buffer);
static uint64_t _poly_check(uint32_t poly, uint32_t bits, uint8_t numbits);
static uint8_t  _determine_polynomial(uint64_t chipsH1, int8_t *start_val);
static uint64_t _hamming_weight(uint64_t bits_in) __attribute__((unused));
static uint32_t _lfsr_index_search(_lfsr_checkpoint_t *checkpoint, uint8_t index, uint32_t bits);
static bool     _check_mocap_interference(uint8_t *arr, uint8_t size);
static void     db_lh2_process_location(db_lh2_t *lh2);
static void     _update_lfsr_checkpoints(uint8_t polynomial, uint32_t bits, uint32_t count, uint8_t sweep);
static uint8_t  _select_sweep(db_lh2_t *lh2, uint8_t polynomial, uint32_t timestamp);
static uint32_t db_lh2_period(uint8_t station);
static uint32_t db_timer_hf_now(uint8_t dev);

//=========================== bsp/lh2_checkpoints.h ============================

static const uint32_t _polynomials[LH2_POLYNOMIAL_COUNT] = {
    0x0001D258,
    0x00017E04,
    0x0001FF6B,
    0x00013F67,
    0x0001B9EE,
    0x000198D1,
    0x000178C7,
    0x00018A55,
    0x00015777,
    0x0001D911,
    0x00015769,
    0x0001991F,
    0x00012BD0,
    0x0001CF73,
    0x0001365D,
    0x000197F5,
    0x000194A0,
    0x0001B279,
    0x00013A34,
    0x0001AE41,
    0x000180D4,
    0x00017891,
    0x00012E64,
    0x00017C72,
    0x00019C6D,
    0x00013F32,
    0x0001AE14,
    0x00014E76,
    0x00013C97,
    0x000130CB,
    0x00013750,
    0x0001CB8D,
};

/// _lfsr_index_table at 228ce9a, polynomial by polynomial, slot by slot
static const uint32_t _lfsr_index_table[LH2_POLYNOMIAL_COUNT][NUM_LSFR_COUNT_CHECKPOINTS] = {
    { 104891, 22375, 89380, 82546, 72190, 94537, 17, 12032,
      34419, 56719, 29233, 17173, 20667, 77346, 44727, 3402,
      49868, 10311, 37782, 18911, 67022, 73904, 58419, 103115,
      36092, 65320, 41248, 32679, 106588, 1726, 6841, 75584,
      101425, 24050, 15502, 42982, 92856, 5160, 85898, 84222,
      27537, 91095, 53274, 70482, 30938, 8573, 87677, 68731,
      97961, 55047, 80804, 39509, 60154, 25783, 46401, 61913,
      96262, 13740, 48081, 79041, 108275, 63605, 99663, 51548 },
    { 99695, 10285, 108302, 51581, 5167, 82518, 53274, 80760,
      91102, 49833, 41, 58429, 68725, 67025, 72157, 106524,
      36092, 63595, 1719, 13769, 18869, 84200, 42944, 25773,
      15466, 22348, 34345, 56689, 32674, 39529, 6853, 73877,
      87694, 12013, 94508, 54972, 92843, 60156, 46425, 17190,
      103089, 3455, 77300, 96231, 29251, 44677, 37800, 89396,
      104837, 27502, 61887, 20629, 70460, 85904, 41247, 8572,
      65298, 30916, 101432, 97989, 24106, 75617, 48153, 79078 },
    { 82521, 32691, 97970, 92766, 39579, 34363, 25783, 61876,
      108264, 46413, 30967, 5154, 10294, 72232, 73918, 18905,
      3427, 63593, 91065, 106528, 67049, 55029, 85944, 84219,
      13714, 87659, 94578, 51540, 43004, 89371, 49819, 70447,
      60183, 75646, 79069, 8579, 101409, 96245, 1725, 29219,
      37781, 17, 103108, 68750, 58473, 24073, 41259, 36073,
      15458, 53277, 77355, 44717, 20596, 99670, 65279, 104895,
      22362, 56765, 48136, 27500, 12050, 6856, 80759, 17210 },
    { 10313, 72196, 41232, 42977, 61847, 65302, 99687, 24044,
      3424, 101405, 34377, 46383, 30951, 106545, 53273, 22366,
      79090, 67039, 70469, 58392, 6897, 97952, 8588, 1736,
      84227, 20627, 37851, 56681, 18937, 55003, 96285, 60172,
      49830, 13727, 36098, 89347, 68751, 27484, 12008, 15483,
      32660, 85943, 17214, 30, 44680, 103141, 51578, 82487,
      25766, 75584, 29176, 91099, 73950, 108267, 104828, 63578,
      87626, 94575, 39513, 77318, 92801, 80787, 5136, 48135 },
    { 60132, 63614, 48144, 10318, 36117, 25779, 99728, 85904,
      18898, 82502, 61842, 44657, 3430, 92799, 68756, 46408,
      89410, 37799, 73871, 17213, 87702, 91091, 5169, 37,
      84238, 13738, 1710, 42960, 41228, 108283, 20628, 6852,
      80746, 103117, 27463, 94520, 39518, 8613, 56739, 75661,
      101370, 29259, 97967, 54981, 79037, 34379, 30963, 12043,
      58477, 22369, 106581, 67027, 53256, 15454, 51547, 24030,
      65317, 77304, 104849, 32631, 96250, 49853, 70472, 72152 },
    { 51571, 20646, 84217, 70471, 49890, 99709, 1678, 73953,
      6907, 34407, 60157, 92830, 29264, 67020, 15466, 85954,
      22360, 108296, 10312, 18892, 94547, 27502, 106567, 5128,
      55031, 41251, 32646, 97969, 75626, 101397, 3430, 37793,
      96211, 87686, 49, 44683, 36079, 58479, 68732, 80743,
      103117, 89377, 63602, 61876, 39530, 13763, 104821, 77381,
      8596, 56716, 65286, 25822, 79020, 82512, 24059, 46385,
      17197, 12028, 72190, 53306, 91108, 48097, 30936, 42957 },
    { 42950, 106575, 82493, 51550, 17210, 70452, 99714, 49834,
      84249, 68762, 46419, 56717, 25773, 15463, 8571, 89332,
      48127, 58445, 12027, 77365, 60154, 3409, 20610, 75635,
      53282, 61871, 34398, 24073, 36115, 41248, 27495, 10334,
      91138, 55010, 96237, 101412, 85922, 5114, 44702, 6914,
      1754, 21, 67047, 13714, 108303, 18906, 103127, 29195,
      30944, 63577, 80788, 92817, 72176, 104838, 65346, 39528,
      73906, 97953, 37814, 79101, 94532, 22350, 32617, 87655 },
    { 77297, 17175, 70460, 27486, 94515, 108291, 53278, 61891,
      3431, 11995, 63596, 37, 99693, 25761, 84221, 15474,
      96226, 44713, 67050, 37786, 91090, 20643, 58418, 42943,
      85932, 1700, 103117, 87609, 65296, 82454, 5166, 32662,
      92811, 72203, 8587, 75596, 24060, 29223, 22345, 73921,
      104875, 97995, 48085, 55006, 41265, 10269, 60184, 68783,
      30954, 106561, 36118, 51554, 56709, 18898, 89385, 34405,
      6870, 80808, 49850, 101427, 13752, 79038, 39530, 46385 },
    { 0, 94551, 103154, 77345, 41253, 13710, 104810, 67051,
      106585, 58399, 10302, 82499, 42979, 25787, 98004, 92795,
      91104, 12074, 24060, 72207, 22345, 101450, 87663, 39532,
      99647, 73895, 68768, 70446, 53290, 46427, 61905, 3407,
      49819, 63583, 80740, 65301, 1684, 6877, 20629, 75598,
      51575, 108258, 32626, 79055, 29196, 36089, 85908, 84242,
      96278, 60139, 44661, 17173, 89379, 30953, 55044, 5149,
      8599, 56699, 15505, 18939, 37811, 34389, 48104, 27541 },
    { 103076, 12028, 6907, 18917, 8592, 27508, 70488, 84191,
      46431, 58414, 68715, 10322, 106544, 30924, 22342, 72214,
      15507, 80732, 51601, 97971, 37827, 85941, 82519, 79014,
      20646, 94516, 60144, 48103, 87646, 13752, 42942, 89390,
      67051, 1771, 99701, 75594, 108283, 5171, 41, 39516,
      32642, 96200, 3434, 29262, 56720, 73906, 77339, 55001,
      41288, 49852, 44679, 92845, 101371, 63593, 34398, 36101,
      24038, 17191, 91139, 53287, 25780, 61908, 104855, 65327 },
    { 68748, 56727, 60150, 70458, 29204, 8607, 79089, 37802,
      46431, 12059, 20605, 108252, 27502, 101441, 42977, 58435,
      103160, 97965, 51532, 84214, 1706, 17191, 65316, 48147,
      55025, 99706, 32649, 36113, 77352, 6876, 73938, 72191,
      3426, 10357, 18907, 63621, 22329, 92765, 61853, 96256,
      104872, 44690, 39548, 82502, 80810, 5145, 41256, 24069,
      53283, 89377, 67067, 85936, 106578, 15481, 13796, 87680,
      49842, 91097, 43, 94527, 34359, 30922, 75621, 25781 },
    { 34336, 36090, 67048, 70460, 27521, 56713, 79065, 49833,
      65307, 75629, 29190, 82496, 39513, 61880, 17167, 20650,
      22365, 72226, 60167, 99693, 103134, 24022, 37794, 80793,
      51533, 1746, 30936, 5158, 91089, 97954, 96246, 6918,
      3449, 55019, 12053, 42977, 8627, 87678, 106558, 41268,
      101396, 108310, 104869, 48127, 18909, 32631, 92789, 73939,
      16, 44694, 53263, 10269, 68772, 89357, 63603, 94517,
      77341, 13749, 84194, 46392, 15471, 25791, 85917, 58450 },
    { 82516, 10311, 63550, 91083, 24061, 68795, 27502, 30919,
      49838, 75649, 34391, 65285, 79050, 85922, 60152, 104838,
      15493, 6862, 8584, 44704, 49, 20626, 97968, 1735,
      80788, 3405, 46415, 89363, 58452, 77339, 87656, 56707,
      70460, 73902, 36047, 67078, 18929, 53277, 108275, 101383,
      61885, 43008, 32651, 48135, 37829, 13764, 54955, 106562,
      94563, 92804, 103096, 5171, 22373, 51587, 12013, 99663,
      17153, 25747, 84216, 29223, 41245, 39502, 96247, 72149 },
    { 92847, 82504, 101392, 36111, 84221, 89331, 61902, 1674,
      12027, 30921, 17196, 65277, 51521, 6861, 106606, 5113,
      44680, 94487, 75638, 8616, 32645, 34372, 25819, 68746,
      67033, 3425, 29226, 63604, 58419, 39527, 104838, 103161,
      87692, 56727, 96246, 97964, 54982, 24038, 80780, 13796,
      72203, 77364, 41275, 73938, 91062, 99671, 48116, 79086,
      37788, 42961, 22347, 60117, 108291, 18879, 70474, 27506,
      85945, 24, 10281, 20628, 46430, 49833, 15490, 53272 },
    { 80781, 1744, 106572, 87684, 91109, 108243, 3422, 104857,
      32652, 46398, 96244, 82544, 53298, 15515, 6832, 30911,
      41257, 8588, 42925, 79066, 84215, 97970, 73920, 27479,
      49818, 48113, 29198, 72159, 75637, 20612, 103132, 5118,
      10306, 60167, 89411, 18882, 36080, 77342, 55030, 68741,
      34409, 25828, 65275, 44701, 101403, 24061, 58389, 92832,
      85889, 13735, 23, 17195, 37824, 12034, 70490, 66983,
      51564, 63620, 56727, 94566, 61870, 99646, 22316, 39544 },
    { 101404, 12041, 15452, 103132, 106583, 92764, 29212, 18933,
      79082, 104834, 80792, 84232, 27537, 63641, 8580, 72180,
      22340, 99695, 46419, 56705, 13721, 89364, 44694, 65333,
      61858, 98011, 58418, 34381, 68708, 87629, 94527, 77351,
      51571, 48131, 6850, 41262, 66994, 17171, 39522, 55004,
      25752, 30933, 60157, 85910, 24051, 1736, 42984, 108283,
      75631, 73938, 36045, 3454, 32630, 49831, 10328, 17,
      5142, 53319, 96265, 91089, 20644, 70510, 37821, 82534 },
    { 82518, 72225, 56727, 85951, 27520, 51593, 34380, 15500,
      48143, 73898, 22355, 94512, 101408, 36062, 8579, 3483,
      53308, 17170, 35, 39535, 29193, 91056, 24089, 32671,
      43005, 6896, 96258, 104846, 44665, 58406, 108263, 10344,
      92790, 20666, 89394, 66985, 54981, 5187, 75606, 46395,
      60184, 12002, 103084, 25775, 1691, 68756, 49869, 13755,
      37809, 77384, 99660, 30914, 63619, 65305, 41286, 106545,
      79052, 70499, 87653, 97952, 61896, 84194, 80742, 18909 },
    { 51596, 3447, 25814, 6879, 36102, 61878, 18938, 91056,
      20622, 96238, 73878, 44690, 99658, 77350, 89366, 5128,
      24064, 85902, 37841, 54984, 80828, 63589, 41257, 104851,
      87688, 8584, 53293, 22332, 56723, 1759, 32628, 92828,
      65315, 39524, 10321, 17200, 60172, 23, 29246, 34386,
      97946, 48124, 11988, 42948, 79104, 94511, 13793, 68795,
      27503, 15480, 70451, 49870, 46416, 67043, 75631, 108285,
      72224, 106577, 84205, 103110, 101386, 58467, 30956, 82468 },
    { 79075, 42928, 39485, 36107, 60146, 68788, 87661, 13790,
      56720, 106514, 58472, 30943, 51565, 108281, 96243, 6886,
      43, 3483, 15439, 63573, 91124, 65314, 99695, 104874,
      97948, 37847, 89345, 17182, 1744, 27469, 41236, 61857,
      92795, 34378, 46424, 18938, 101397, 94549, 67036, 82514,
      32635, 22309, 12054, 103115, 72142, 49837, 48147, 10321,
      84192, 44691, 73908, 8555, 20626, 85987, 75631, 5149,
      77358, 80768, 55004, 25791, 53244, 70440, 29180, 24021 },
    { 46364, 8586, 55013, 12, 53239, 37816, 44654, 3468,
      70458, 89360, 92851, 36063, 22353, 30922, 80741, 91096,
      103112, 67001, 1686, 34337, 32672, 48144, 27478, 68743,
      73866, 84182, 42959, 41287, 97942, 24073, 101435, 96244,
      12025, 65325, 18916, 60178, 51583, 39533, 61896, 99682,
      5144, 58441, 106596, 72188, 20635, 82509, 87663, 63605,
      17219, 29185, 49818, 25779, 15463, 13728, 10324, 108305,
      75616, 104877, 85903, 6883, 79059, 56706, 94499, 77371 },
    { 25804, 101376, 84232, 39547, 46418, 5130, 6895, 32666,
      79073, 1735, 67017, 80768, 97980, 12004, 3447, 77298,
      91096, 56703, 96223, 61843, 36112, 41225, 73862, 37809,
      48125, 36, 87675, 103123, 60191, 49825, 104850, 18926,
      58417, 22325, 53277, 44674, 29217, 85965, 27515, 68719,
      63562, 108325, 70459, 42923, 92816, 8585, 13752, 82524,
      89402, 24096, 99699, 30934, 65324, 17193, 72181, 75598,
      51551, 20614, 94501, 10309, 15446, 106541, 34391, 55000 },
    { 10341, 87613, 97984, 39534, 92810, 13732, 37776, 46370,
      25777, 103139, 18897, 60170, 72171, 75660, 12051, 84246,
      49843, 65326, 27500, 58424, 44666, 101423, 67016, 85943,
      8571, 96249, 3408, 30956, 20613, 51568, 6875, 41266,
      17209, 104874, 77353, 79039, 70429, 22358, 108264, 82522,
      63582, 36100, 29188, 73931, 34367, 1717, 56754, 15466,
      42990, 32636, 91062, 94525, 19, 48132, 5207, 89351,
      106550, 24053, 80788, 54980, 68768, 53320, 61925, 99688 },
    { 82509, 15477, 103090, 92774, 61849, 6907, 29225, 8573,
      79096, 14, 97948, 94517, 56681, 67049, 80774, 85979,
      12036, 77318, 58436, 75624, 68772, 63608, 36087, 72207,
      91083, 17230, 27492, 99688, 34361, 5186, 32696, 70444,
      44690, 20622, 49852, 106515, 18944, 46427, 1696, 39567,
      51534, 22301, 10307, 65326, 60108, 30938, 53267, 89334,
      25783, 24058, 48136, 3414, 73909, 87639, 43008, 84233,
      108258, 41275, 96227, 54999, 101417, 13766, 104835, 37799 },
    { 58414, 70453, 108296, 24048, 49876, 85902, 32633, 39519,
      41208, 77326, 92795, 18926, 22354, 89368, 94522, 1729,
      96250, 63606, 61899, 54975, 99676, 80772, 17, 53237,
      15454, 65304, 10304, 25795, 101385, 75595, 48079, 44689,
      82508, 34394, 87656, 29189, 84200, 12031, 17186, 72232,
      3461, 30896, 13751, 46408, 8589, 106596, 79079, 36088,
      56754, 51607, 42977, 104823, 68771, 73894, 66992, 103153,
      5166, 91071, 60145, 6888, 37812, 97942, 20627, 27485 },
    { 94577, 11, 44702, 17189, 91089, 58442, 67054, 96282,
      34380, 12005, 53265, 39544, 61894, 20624, 103132, 48101,
      85935, 79022, 108270, 3408, 32666, 42971, 106553, 10344,
      37772, 6874, 104885, 8587, 97983, 80774, 49821, 46390,
      77342, 51538, 29220, 101409, 68779, 13713, 75627, 15471,
      70424, 1725, 41240, 87646, 18913, 82511, 27535, 99668,
      89386, 73904, 25781, 60153, 92807, 72179, 30935, 36062,
      5156, 22333, 55019, 56716, 65281, 63631, 24058, 84221 },
    { 30946, 103087, 106578, 36084, 92818, 1704, 48127, 67021,
      72206, 97986, 14, 101389, 12033, 24079, 44700, 18898,
      91082, 77315, 51600, 108269, 82477, 80793, 8608, 46370,
      25744, 70503, 75607, 15492, 53253, 39566, 58438, 85934,
      94540, 65317, 22333, 61903, 17171, 13740, 87648, 49863,
      10311, 63567, 20660, 32677, 29226, 68752, 73915, 42982,
      99692, 27477, 37807, 41245, 104878, 6869, 60155, 5131,
      79053, 56721, 96248, 84259, 89417, 34385, 3452, 55011 },
    { 92853, 96241, 10298, 13775, 37820, 17226, 108323, 103135,
      48105, 14, 63610, 79065, 54973, 70451, 41272, 49816,
      3390, 42936, 91143, 56742, 24064, 12078, 60193, 5110,
      30954, 46393, 80741, 27459, 34374, 15466, 61853, 89369,
      8633, 51535, 101372, 85918, 36072, 84210, 6864, 44708,
      99695, 32654, 72206, 18880, 73862, 75624, 25786, 67049,
      22391, 29197, 53265, 1713, 65319, 77366, 68753, 39501,
      94532, 58457, 104802, 82537, 97991, 106599, 87643, 20622 },
    { 60151, 41273, 25766, 58435, 70479, 13, 73918, 68778,
      82522, 49870, 84213, 36098, 85912, 13755, 65294, 29214,
      108285, 24055, 79088, 106571, 54999, 77360, 5190, 63576,
      61874, 1711, 103112, 104879, 15513, 96237, 91085, 34361,
      6849, 20618, 97968, 30973, 53300, 46408, 10316, 27469,
      22360, 80776, 3430, 32665, 12019, 37769, 56724, 51548,
      92781, 44721, 101414, 94541, 89372, 87652, 17170, 42970,
      99705, 18907, 75625, 67020, 8597, 48171, 39532, 72153 },
    { 15434, 103095, 39544, 12019, 68773, 27501, 94495, 46372,
      10307, 72209, 29210, 65309, 87633, 20612, 34380, 24096,
      6847, 32635, 48134, 80810, 104870, 82513, 13763, 70477,
      99695, 53263, 44721, 63637, 60124, 27, 25769, 67038,
      75585, 89350, 56679, 84206, 92811, 42960, 3465, 108250,
      5152, 98010, 30893, 73885, 96217, 1699, 36062, 91063,
      77378, 79053, 101423, 58452, 37796, 18913, 106543, 85903,
      8598, 22353, 55047, 17159, 41262, 61912, 51539, 49808 },
    { 82506, 96232, 36083, 20661, 15465, 43, 89388, 70459,
      99722, 87637, 79022, 6888, 103079, 13704, 37808, 29259,
      32655, 80796, 46445, 18863, 60187, 73913, 77348, 44718,
      24043, 53264, 5170, 56714, 58428, 61904, 27522, 92795,
      48114, 94542, 63592, 42954, 75668, 104842, 106529, 10318,
      17161, 84221, 11991, 108285, 30927, 91052, 97996, 25793,
      49824, 22301, 3469, 39544, 8612, 67015, 34335, 41233,
      55011, 65336, 101392, 72201, 51524, 85939, 1731, 68731 },
    { 22385, 11, 106601, 27537, 36101, 97930, 30961, 1708,
      72232, 48142, 10306, 17230, 67034, 3444, 101398, 96221,
      103155, 49868, 61891, 92855, 75670, 70462, 68751, 41211,
      65326, 39522, 58428, 51588, 8634, 46357, 94517, 63558,
      37805, 73921, 15477, 87660, 12021, 56684, 42990, 44661,
      32661, 25762, 84205, 85968, 99697, 6908, 108262, 89337,
      20598, 5175, 60183, 24022, 79043, 82504, 80786, 55031,
      18932, 29242, 104876, 13735, 91066, 53279, 34406, 77335 },
    { 72162, 22338, 49849, 25776, 82504, 42968, 60180, 39526,
      70436, 3444, 30925, 56741, 8591, 101402, 103151, 5135,
      32660, 36113, 65295, 1714, 67026, 106559, 80787, 104873,
      68762, 43, 51552, 92826, 63577, 44645, 108245, 27486,
      89379, 18907, 46407, 17204, 12036, 97960, 61863, 6863,
      85948, 29221, 91094, 10302, 34388, 96267, 48140, 84224,
      58470, 54969, 73927, 75637, 37812, 79060, 77330, 15453,
      13780, 94529, 20634, 87695, 24035, 53307, 99680, 41223 },
};

/// _lfsr_hash_table at 228ce9a: the LFSR state at each index above, filled by lh2_ref_init()
static uint32_t _lfsr_hash_table[LH2_POLYNOMIAL_COUNT][NUM_LSFR_COUNT_CHECKPOINTS];

//=========================== bsp/nrf/lh2_decoder.c ============================

#define FUZZY_CHIP                             0xFF  ///< not sure what this is about
#define POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD 0     ///< tolerate no errors in received data
// #define POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD 4                             ///< initial threshold of polynomial error
#define HASH_TABLE_BITS 6                             ///< How many bits will be used for the hashtable for the _end_buffers
#define HASH_TABLE_MASK ((1 << HASH_TABLE_BITS) - 1)  ///< Mask selecting the HAS_TABLE_BITS least significant bits

// Define variables useful for debuging
typedef uint32_t lfsr_17bits_t;

uint64_t _demodulate_light(uint8_t *sample_buffer) {  // bad input variable name!!
    // TODO: rename sample_buffer
    // TODO: make it a void and have chips be a modified pointer thingie
    // FIXME: there is an edge case where I throw away an initial "1" and do not count it in the bit-shift offset, resulting in an incorrect error of 1 in the LFSR location
    uint8_t chip_index;
    uint8_t zccs_1[256];  // indexed by the uint8_t chip_index, which counts past 128 on a busy capture
    uint8_t chips1[128];  // TODO: give this a better name.
    uint8_t temp_byte_N;  // TODO: bad variable name "temp byte"
    uint8_t temp_byte_M;  // TODO: bad variable name "temp byte"

    // initialize loop variables
    int      jj = 0;
    int      kk = 0;
    uint64_t gg = 0;

    // initialize temporary "ones counter" variable that counts consecutive ones
    int ones_counter = 0;

    // initialize result:
    uint64_t chipsH1 = 0;

    // FIND ZERO CROSSINGS
    // An entry no crossing reaches keeps this count, which thresholds to a
    // zero chip, so the tail past the last crossing is deterministic.
    memset(zccs_1, 0xFF, sizeof(zccs_1));
    chip_index         = 0;
    zccs_1[chip_index] = 0x01;

    // for loop over bytes of the SPI buffer (jj), nested with a for loop over bits in each byte (ii)
    for (jj = 0; jj < 128; jj++) {
        // edge case - check if last bit (LSB) of previous byte is the same as first bit (MSB) of current byte
        // if it is not, increment chip_index and reset count
        if (jj != 0) {
            temp_byte_M = (sample_buffer[jj - 1]) & (0x01);   // previous byte's LSB
            temp_byte_N = (sample_buffer[jj] >> 7) & (0x01);  // current byte's MSB
            if (temp_byte_M != temp_byte_N) {
                chip_index++;
                zccs_1[chip_index] = 1;
            } else {
                zccs_1[chip_index] += 1;
            }
        }
        // detect intra-byte transitions with a single XOR: bit k is set when bit(k+1) != bit(k)
        uint8_t transitions = sample_buffer[jj] ^ (sample_buffer[jj] >> 1);
        for (int bit = 6; bit >= 0; bit--) {
            if ((transitions >> bit) & 1) {
                chip_index++;
                zccs_1[chip_index] = 1;
            } else {
                zccs_1[chip_index]++;
            }
        }
    }

    // threshold the zero crossings into: likely one chip, likely two zero chips, or fuzzy
    for (jj = 0; jj < 128; jj++) {
        // not memory efficient, but ok for readability, turn ZCCS into chips by thresholding
        if (zccs_1[jj] >= 5) {
            chips1[jj] = 0;  // it's a very likely zero
        } else if (zccs_1[jj] <= 3) {
            chips1[jj] = 1;  // it's a very likely one
        } else {
            chips1[jj] = FUZZY_CHIP;  // fuzzy
        }
    }
    // final bit is bugged, make it fuzzy:
    // chips1[127] = 0xFF;

    // DEMODULATION:
    // basic principles, in descending order of importance:
    //  1) an odd number of ones in a row is not allowed - this must be avoided at all costs
    //  2) finding a solution to #1 given a set of data is quite cumbersome without certain assumptions
    //    a) a fuzzy before an odd run of 1s is almost always a 1
    //    b) a fuzzy between two even runs of 1s is almost always a 0
    //    c) a fuzzy after an even run of 1s is usually a a 0
    //  3) a detected 1 is rarely wrong, but detected 0s can be, this is especially common in low-SNR readings
    //    exception: if the first bit is a 1 it is NOT reliable because the capture is asynchronous
    //  4) this is not perfect, but the earlier the chip, the more likely that it is correct. Polynomials can be used to fix bit errors later in the reading
    // known bugs/issues:
    //  1) if there are many ones at the very beginning of the reading, the algorithm will mess it up
    //  2) in some instances, the count value will be off by approximately 5, the origin of this bug is unknown at the moment
    // DEMODULATE PACKET:

    // reset variables:
    kk           = 0;
    ones_counter = 0;
    jj           = 0;
    for (jj = 0; jj < 128;) {      // TODO: 128 is such an easy magic number to get rid of...
        if (chips1[jj] == 0x00) {  // zero, keep going, reset state
            jj++;
            ones_counter = 0;
            if (jj >= 128) {
                break;
            }
        }
        if (chips1[jj] == 0x01) {  // one, keep going, keep track of the # of ones
                                   // k_msleep(10);
            if (jj == 0) {         // edge case - first chip = 1 is unreliable, do not increment 1s counter
                jj++;
            } else {
                jj           = jj + 1;
                ones_counter = ones_counter + 1;
            }
            if (jj >= 128) {
                break;
            }
        }

        if ((jj == 127) & (chips1[jj] == FUZZY_CHIP)) {
            chips1[jj] = 0x00;
        } else if ((chips1[jj] == FUZZY_CHIP) & (ones_counter == 0)) {  // fuzz after a zero
                                                                        // k_msleep(10);
            if (chips1[jj + 1] == 0) {                                  // zero then fuzz then zero -> fuzz is a zero
                jj++;
                chips1[jj - 1] = 0;
            } else if (chips1[jj + 1] == FUZZY_CHIP) {  // zero then fuzz then fuzz -> just move on, you're probably screwed
                // k_msleep(10);
                jj += 2;
            } else if (chips1[jj + 1] == 1) {  // zero then fuzz then one -> investigate
                kk           = 1;
                ones_counter = 0;
                while (jj + kk < 128 && chips1[jj + kk] == 1) {
                    ones_counter++;
                    kk++;
                }
                if (ones_counter % 2 == 1) {  // fuzz -> odd ones, the fuzz is a 1
                    jj++;
                    chips1[jj - 1] = 1;
                    ones_counter   = 1;
                } else if (ones_counter % 2 == 0) {  // fuzz -> even ones, move on for now, it's indeterminate
                    jj++;
                    ones_counter = 0;  // temporarily treat as a 0 for counting purposes
                } else {               // catch statement
                    jj++;
                }
            }
        } else if ((chips1[jj] == FUZZY_CHIP) & (ones_counter != 0)) {  // ones then fuzz
                                                                        // k_msleep(10);
            if ((ones_counter % 2 == 0) & (chips1[jj + 1] == 0)) {      // even ones then fuzz then zero, fuzz is a zero
                jj++;
                chips1[jj - 1] = 0;
                ones_counter   = 0;
            }
            if ((ones_counter % 2 == 0) && (jj + 1 < 128) && (chips1[jj + 1] != 0)) {  // even ones then fuzz then not zero - investigate
                if (chips1[jj + 1] == 1) {                                             // subsequent bit is a 1
                    kk = 1;
                    while (jj + kk < 128 && chips1[jj + kk] == 1) {
                        ones_counter++;
                        kk++;
                    }
                    if (ones_counter % 2 == 1) {  // indicates an odd # of 1s, so the fuzzy has to be a 1
                        jj++;
                        chips1[jj - 1] = 1;
                        ones_counter   = 1;              // not actually 1, but it's ok for modulo purposes
                    } else if (ones_counter % 2 == 0) {  // even ones -> fuzz -> even ones, indeterminate
                        jj++;
                        ones_counter = 0;
                    }
                } else if (chips1[jj + 1] == FUZZY_CHIP) {  // subsequent bit is a fuzzy - skip for now...
                    jj++;
                }
            } else if ((ones_counter % 2 == 1) && (chips1[jj + 1] == FUZZY_CHIP)) {  // odd ones then fuzz then fuzz, fuzz is 1 then 0
                jj += 2;
                chips1[jj - 1] = 0;
                chips1[jj - 2] = 1;
                ones_counter   = 0;
            } else if ((ones_counter % 2 == 1) && (chips1[jj + 1] != 0)) {  // odd ones then fuzz then not zero - the fuzzy has to be a 1
                jj++;
                ones_counter++;
                chips1[jj - 1] = 1;
            } else {  // catch statement
                jj++;
            }
        }
    }
    // finish up demodulation, pick off straggling fuzzies and odd runs of 1s
    ones_counter = 0;
    for (jj = 0; jj < 128;) {
        if (chips1[jj] == 0x00) {                   // zero, keep going, reset state
            if (ones_counter % 2 == 1) {            // implies an odd # of 1s
                chips1[jj - ones_counter - 1] = 1;  // change the bit before the run of 1s to a 1 to make it even
            }
            jj++;
            ones_counter = 0;
        } else if (chips1[jj] == 0x01) {  // one, keep going, keep track of the # of ones
            if (jj == 0) {                // edge case - first chip = 1 is unreliable, do not increment 1s counter
                jj++;
            } else {
                jj           = jj + 1;
                ones_counter = ones_counter + 1;
            }
        } else if (chips1[jj] == FUZZY_CHIP) {
            // if (ones_counter==0) { // fuzz after zeros, if the next chip is a 1, make it a 1, else make it a zero
            //     if (chips1[jj+1]==1) {
            //         jj+1;
            //         chips1[jj-1] = 1;
            //         ones_counter++;
            //     }
            //     else {
            //         jj++;
            //     }
            // }  <---- this is commented out because this is a VERY rare edge case and seems to be causing occasional problems w/ otherwise clean packets
            if ((ones_counter != 0) & (ones_counter % 2 == 0)) {  // fuzz after even ones - at this point this is almost always a 0
                jj++;
                chips1[jj - 1] = 0;
                ones_counter   = 0;
            } else if (ones_counter % 2 == 1) {  // fuzz after odd ones - exceedingly uncommon at this point, make it a 1
                jj++;
                chips1[jj - 1] = 1;
                ones_counter++;
            } else {  // catch statement
                jj++;
            }
        } else {  // catch statement
            jj++;
        }
    }

    // next step in demodulation: take the resulting array of 1 and 0 chips and put them into a single 64-bit unsigned int
    // this is primarily for easy manipulation for polynomial searching
    chip_index = 0;  // TODO: rename "chip index" it's not descriptive
    chipsH1    = 0;
    gg         = 0;    // looping/while break indicating variable, reset to 0
    while (gg < 64) {  // very last one - make all remaining fuzzies 0 and load it into two 64-bit longs
        if (chip_index > 127) {
            break;
        }
        if ((chip_index == 0) & (chips1[chip_index] == 0x01)) {  // first bit is a 1 - ignore it
            chip_index = chip_index + 1;
        } else if ((chip_index == 0) & (chips1[chip_index] == FUZZY_CHIP)) {  // first bit is fuzzy - ignore it
            chip_index = chip_index + 1;
        } else if (gg == 63) {  // load the final bit
            if (chips1[chip_index] == 0) {
                chipsH1 &= 0xFFFFFFFFFFFFFFFE;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == FUZZY_CHIP) {
                chipsH1 &= 0xFFFFFFFFFFFFFFFE;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == 0x01) {
                chipsH1 |= 0x0000000000000001;
                gg         = gg + 1;
                chip_index = chip_index + 2;
            }
        } else {  // load the bit in!!
            if (chips1[chip_index] == 0) {
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == FUZZY_CHIP) {
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == 0x01) {
                chipsH1 |= 0x0000000000000001;
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 2;
            }
        }
    }
    return chipsH1;
}

uint64_t _poly_check(uint32_t poly, uint32_t bits, uint8_t numbits) {
    uint64_t bits_out      = 0;
    uint8_t  shift_counter = 1;
    uint8_t  b1            = 0;
    uint32_t buffer        = bits;   // mask to prevent bit overflow
    poly &= 0x00001FFFF;             // mask to prevent silliness
    bits_out |= buffer;              // initialize 17 LSBs of result
    bits_out &= 0x00000000FFFFFFFF;  // mask because I didn't want to re-cast the buffer

    while (shift_counter <= numbits) {
        bits_out = bits_out << 1;  // shift left (forward in time) by 1

        b1     = __builtin_popcount(buffer & poly) & 0x01;  // mask the buffer w/ the selected polynomial
        buffer = ((buffer << 1) | b1) & (0x0001FFFF);

        bits_out |= ((b1) & (0x01));  // put result of the XOR operation into the new bit
        shift_counter++;
    }
    return bits_out;
}

uint8_t _determine_polynomial(uint64_t chipsH1, int8_t *start_val) {
    // check which polynomial the bit sequence is part of
    // TODO: make function a void and modify memory directly
    // TODO: rename chipsH1 to something relevant... like bits?

    // Handle the edgecase of a full zero input
    if (chipsH1 == 0x00) {
        return LH2_POLYNOMIAL_ERROR_INDICATOR;
    }

    *start_val = 8;  // TODO: remove this? possible that I modify start value during the demodulation process

    int32_t  bits_N_for_comp                      = 47 - *start_val;
    uint32_t bit_buffer1                          = (uint32_t)(((0xFFFF800000000000) & chipsH1) >> 47);
    uint64_t bits_from_poly[LH2_POLYNOMIAL_COUNT] = { 0 };
    uint8_t  weights[LH2_POLYNOMIAL_COUNT]        = { 0 };
    uint8_t  selected_poly                        = LH2_POLYNOMIAL_ERROR_INDICATOR;  // initialize to error condition
    uint8_t  min_weight_idx                       = LH2_POLYNOMIAL_ERROR_INDICATOR;
    uint64_t min_weight                           = LH2_POLYNOMIAL_ERROR_INDICATOR;
    uint64_t bits_to_compare                      = 0;
    int32_t  threshold                            = POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD;

#if defined(LH2_MOCAP_FILTER)
    // tighten threshold if we expect MoCap interference
    threshold = 0;
#endif

    // try polynomial vs. first buffer bits
    // this search takes 17-bit sequences and runs them forwards through the polynomial LFSRs.
    // if the remaining detected bits fit well with the chosen 17-bit sequence and a given polynomial, it is treated as "correct"
    // in case of bit errors at the beginning of the capture, the 17-bit sequence is shifted (to a max of 8 bits)
    // in case of bit errors at the end of the capture, the ending bits are removed (to a max of
    // removing bits reduces the threshold correspondingly, as incorrect packet detection will cause a significant delay in location estimate

    // run polynomial search on the first capture
    while (1) {

        // TODO: do this math stuff in multiple operations to: (a) make it readable (b) ensure order-of-execution
        bit_buffer1     = (uint32_t)(((0xFFFF800000000000 >> (*start_val)) & chipsH1) >> (64 - 17 - (*start_val)));
        bits_to_compare = (chipsH1 & (0xFFFFFFFFFFFFFFFF << (64 - 17 - (*start_val) - bits_N_for_comp)));
        // reset the minimum polynomial match found
        min_weight_idx = LH2_POLYNOMIAL_ERROR_INDICATOR;
        min_weight     = LH2_POLYNOMIAL_ERROR_INDICATOR;
        // Check against all the known polynomials
        for (uint8_t i = 0; i < LH2_POLYNOMIAL_COUNT; i++) {
            bits_from_poly[i] = (((_poly_check(_polynomials[i], bit_buffer1, bits_N_for_comp)) << (64 - 17 - (*start_val) - bits_N_for_comp)) | (chipsH1 & (0xFFFFFFFFFFFFFFFF << (64 - (*start_val)))));
            weights[i]        = __builtin_popcountll(bits_from_poly[i] ^ bits_to_compare);
            // Keep track of the minimum weight value and which polinimial generated it.
            if (weights[i] < min_weight) {
                min_weight_idx = i;
                min_weight     = weights[i];
            }
        }

        // If you found a sufficiently good value, then return which polinomial generated it
        if (min_weight <= (uint64_t)threshold) {
            selected_poly = min_weight_idx;
            break;
            // match failed, try again removing bits from the end
        } else if (*start_val > 8) {
            *start_val      = 8;
            bits_N_for_comp = bits_N_for_comp - 9;
            if (threshold > 2) {
                threshold = threshold - 1;
            } else if (threshold == 2) {  // keep threshold at ones, but you're probably screwed with an unlucky bit error
                threshold = 2;
            }
        } else {
            *start_val = *start_val + 1;
            bits_N_for_comp -= 1;
        }

        // too few bits to reliably compare, give up
        if (bits_N_for_comp < 19) {
            selected_poly = LH2_POLYNOMIAL_ERROR_INDICATOR;  // mark the poly as "wrong"
            break;
        }
    }
    return selected_poly;
}

uint64_t _hamming_weight(uint64_t bits_in) {  // TODO: bad name for function? or is it, it might be a good name for a function, because it describes exactly what it does
    uint64_t weight = bits_in;
    weight          = weight - ((weight >> 1) & 0x5555555555555555);                         // find # of 1s in every 2-bit block
    weight          = (weight & 0x3333333333333333) + ((weight >> 2) & 0x3333333333333333);  // find # of 1s in every 4-bit block
    weight          = (weight + (weight >> 4)) & 0x0F0F0F0F0F0F0F0F;                         // find # of 1s in every 8-bit block
    weight          = (weight + (weight >> 8)) & 0x00FF00FF00FF00FF;                         // find # of 1s in every 16-bit block
    weight          = (weight + (weight >> 16)) & 0x0000FFFF0000FFFF;                        // find # of 1s in every 32-bit block
    weight          = (weight + (weight >> 32));                                             // add the two 32-bit block results together
    weight          = weight & 0x000000000000007F;                                           // mask final result, max value of 64, 0'b01000000
    return weight;
}

uint32_t _lfsr_index_search(_lfsr_checkpoint_t *checkpoint, uint8_t index, uint32_t bits) {

    lfsr_17bits_t bits_local  = bits & 0x0001FFFF;  // initialize buffer to initial bits, masked
    lfsr_17bits_t buffer_down = bits_local;
    lfsr_17bits_t buffer_up   = bits_local;

    uint32_t count_down  = 0;
    uint32_t count_up    = 0;
    int32_t  count_final = 0;  // There is a chance for overflow near the roll-over point of the lfsr sequence (sequence very high (>120k), hitting a very low check point (~20))
    uint32_t b17         = 0;
    uint32_t b1          = 0;
    uint32_t masked_buff = 0;
    uint8_t  hash_down   = 0;
    uint8_t  hash_up     = 0;
    bool     success     = false;  // True if the LFSR search produced a valid result

    // Copy const variables (Flash) into local variables (RAM) to speed up execution.
    uint32_t _lfsr_hash_table_local[NUM_LSFR_COUNT_CHECKPOINTS]  = { 0 };
    uint32_t _lfsr_index_table_local[NUM_LSFR_COUNT_CHECKPOINTS] = { 0 };
    uint32_t polynomials_local                                   = _polynomials[index];
    for (size_t i = 0; i < NUM_LSFR_COUNT_CHECKPOINTS; i++) {
        _lfsr_hash_table_local[i]  = _lfsr_hash_table[index][i];
        _lfsr_index_table_local[i] = _lfsr_index_table[index][i];
    }

    // Start the iterative search
    while (buffer_up != 0x00 && buffer_down != 0x00) {  // Check that the count has not fallen into an invalid state    {

        //
        // CHECKPOINT CHECKING
        //

        // Check lfsr backward count against precomputed checkpoints
        hash_down = (buffer_down >> 11) & HASH_TABLE_MASK;
        if (buffer_down == _lfsr_hash_table_local[hash_down]) {
            count_final = count_down + _lfsr_index_table_local[hash_down];
            success     = true;  // mark the success of the search
            break;
        }

        // Check lfsr forward count against precomputed checkpoints
        hash_up = (buffer_up >> 11) & HASH_TABLE_MASK;
        if (buffer_up == _lfsr_hash_table_local[hash_up]) {
            count_final = _lfsr_index_table_local[hash_up] - count_up;
            success     = true;  // mark the success of the search
            break;
        }

        // Check the dynamical checkpoints, backward
        // Sweep 0
        if (buffer_down == checkpoint->bits[index][0]) {
            count_final = count_down + checkpoint->count[index][0];
            success     = true;  // mark the success of the search
            break;
        }
        // Sweep 1
        if (buffer_down == checkpoint->bits[index][1]) {
            count_final = count_down + checkpoint->count[index][1];
            success     = true;  // mark the success of the search
            break;
        }

        // Check the dynamical checkpoints, forward
        // Sweep 0
        if (buffer_up == checkpoint->bits[index][0]) {
            count_final = checkpoint->count[index][0] - count_up;
            success     = true;  // mark the success of the search
            break;
        }
        // Sweep 1
        if (buffer_up == checkpoint->bits[index][1]) {
            count_final = checkpoint->count[index][1] - count_up;
            success     = true;  // mark the success of the search
            break;
        }

        //
        // LSFR UPDATE
        //

        // LSFR backward update
        b17         = buffer_down & 0x00000001;                                                      // save the "newest" bit of the buffer
        buffer_down = (buffer_down & (0x0001FFFE)) >> 1;                                             // shift the buffer right, backwards in time
        masked_buff = (buffer_down) & (polynomials_local);                                           // mask the buffer w/ the selected polynomial
        buffer_down = buffer_down | (((__builtin_popcount(masked_buff) ^ b17) & 0x00000001) << 16);  // This weird line propagates the LSFR one bit into the past
        count_down++;

        // LSFR forward update
        b1        = __builtin_popcount(buffer_up & polynomials_local) & 0x01;  // mask the buffer w/ the selected polynomial
        buffer_up = ((buffer_up << 1) | b1) & (0x0001FFFF);
        count_up++;
    }

    // Return the found lfsr value
    if (success) {
        // Handle overflows
        if (count_final < 0) {
            // wrap around the number
            count_final += 131071;  // 2^17 -1 (the full lenght of the sequence)
        }
        // Turn result back to unsigned before returning
        return (uint32_t)count_final;
    } else {
        return LH2_LFSR_SEARCH_ERROR_INDICATOR;
    }
}

bool _check_mocap_interference(uint8_t *arr, uint8_t size) {

    // Qualysis Mocap cameras pulse IR light modulated with a regular square wave at 1Mhz.
    // At the 32Mhz speed we sample the SPI, that corresponds to an alternating 0xFF,0xFF,0xFF,0x00,0x00,0x00 pattern

    // Check only the bottom half of the array, that should be enough to catch an error.
    for (int i = 0; i < size / 2; i++) {
        // Check for 3 consecutive 0xFF
        if (arr[i] == 0xFF && arr[i + 1] == 0xFF && arr[i + 2] == 0xFF) {
            return true;  // Error for 3 consecutive 0xFF
        }

        // Check for 3 consecutive 0x00
        if (arr[i] == 0x00 && arr[i + 1] == 0x00 && arr[i + 2] == 0x00) {
            return true;  // Error for 3 consecutive 0x00
        }
    }
    return false;  // No error found
}

//=========================== bsp/nrf/lh2_default.c ============================

#define SPI_BUFFER_SIZE               64          ///< Size of buffers used for SPI communications
#define LH2_LOCATION_ERROR_INDICATOR  0xFFFFFFFF  ///< indicate the location value is false
#define LH2_MAX_DATA_VALID_TIME_US    2000000     //< Data older than this is considered outdate and should be erased (in microseconds)
#define LH2_SWEEP_PERIOD_THRESHOLD_US 1000        ///< How close a LH2 pulse must arrive relative to LH2_SWEEP_PERIOD_US, to be considered the same type of sweep (first sweep or second second). (in microseconds)
#define LH2_TIMER_DEV                 2           ///< Timer device used for LH2

/// The hook ring: holds the one capture lh2_ref_feed() passes in
typedef struct {
    uint8_t  buffer[SPI_BUFFER_SIZE];
    uint32_t timestamp;
    uint8_t  count;
} lh2_ring_buffer_t;

typedef struct {
    lh2_ring_buffer_t  data;        ///< the hook ring
    _lfsr_checkpoint_t checkpoint;  ///< Dynamic checkpoints for the lsfr index search
} lh2_vars_t;

static lh2_vars_t _lh2_vars;
static uint32_t   _now_us;

static bool _get_from_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t *timestamp);

///< List of rotational periods (in microseconds) of the lighthouse basestation in all its 16 modes.
static const uint16_t _lh2_sweep_period_us[LH2_BASESTATION_COUNT] = {
    19979,
    19938,
    19854,
    19771,
    19729,
    19646,
    19604,
    19563,
    19521,
    19354,
    19146,
    18979,
    18896,
    18771,
    18604,
    18479,
};

///< Encodes in two bits which sweep slot of a particular basestation is empty, (1 means data, 0 means empty)
typedef enum {
    LH2_SWEEP_BOTH_SLOTS_EMPTY,   ///< Both sweep slots are empty
    LH2_SWEEP_SECOND_SLOT_EMPTY,  ///< Only the second sweep slot is empty
    LH2_SWEEP_FIRST_SLOT_EMPTY,   ///< Only the first sweep slot is empty
    LH2_SWEEP_BOTH_SLOTS_FULL,    ///< Both sweep slots are filled with raw data
} db_lh2_sweep_slot_state_t;

void db_lh2_process_location(db_lh2_t *lh2) {
    // There is no TS4231 data to process, return early.
    if (_lh2_vars.data.count == 0) {
        return;
    }

    //*********************************************************************************//
    //                              Prepare Raw Data                                   //
    //*********************************************************************************//

    // Get value before it's overwritten by the ringbuffer.
    uint8_t temp_spi_bits[SPI_BUFFER_SIZE * 2] = { 0 };  // The temp buffer has to be 128 long because _demodulate_light() expects it to be so
                                                         // Making it smaller causes a hardfault
                                                         // I don't know why, the SPI buffer is clearly 64bytes long.
                                                         // should ask fil about this

    // stop the interruptions while you're reading the data.
    uint32_t temp_timestamp = 0;  // default timestamp
    if (!_get_from_spi_ring_buffer(&_lh2_vars.data, temp_spi_bits, &temp_timestamp)) {
        return;
    }

// Check if Qualysis Mocap data is interfering with the SPI capture
#if defined(LH2_MOCAP_FILTER)
    if (_check_mocap_interference(temp_spi_bits, SPI_BUFFER_SIZE)) {
        return;  // if a qualysis pulse caused a false spi trigger, leave the function.
    }
#endif

    // perform the demodulation received packets
    // convert the SPI reading to bits via zero-crossing counter demodulation and differential/biphasic manchester decoding.
    uint64_t temp_bits_sweep = _demodulate_light(temp_spi_bits);

    // figure out which polynomial the data belongs  to
    int8_t  temp_bit_offset          = 0;  // default offset
    uint8_t temp_selected_polynomial = _determine_polynomial(temp_bits_sweep, &temp_bit_offset);

    // If there was an error with the polynomial, leave without updating anything
    if (temp_selected_polynomial == LH2_POLYNOMIAL_ERROR_INDICATOR) {
        return;
    }

    // Figure out in which of the two sweep slots we should save the new data.
    uint8_t sweep = _select_sweep(lh2, temp_selected_polynomial, temp_timestamp);

    // Compute which basestation the sweep came from (polynomial 0,1 must map to LH0, 2,3 to LH1, etc... This can be accomplish by  integer-dividing the selected poly in 2, a shift >> accomplishes this.)
    uint8_t basestation = temp_selected_polynomial >> 1;

    //*********************************************************************************//
    //                             Compute LFSR Position                               //
    //*********************************************************************************//

    // Select the valid bits of the lfsr by applying the offset (he first few bits might be invalid, as detected by _determine_polynomial())
    uint32_t temp_lfsr_bits = temp_bits_sweep >> (47 - temp_bit_offset);

    // Sanity check, make sure you don't start the LFSR search with a bit-sequence full of zeros.
    if (temp_lfsr_bits == 0x000000) {
        // Mark the data as wrong and keep going
        lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }

    // Compute the lfsr location.
    uint32_t temp_lfsr_loc = _lfsr_index_search(&_lh2_vars.checkpoint,
                                                temp_selected_polynomial,
                                                temp_lfsr_bits);

    // Check that the count didn't fall on an illegal value
    if (temp_lfsr_loc != LH2_LFSR_SEARCH_ERROR_INDICATOR) {
        // Save a new dynamic checkpoint
        _update_lfsr_checkpoints(temp_selected_polynomial, temp_lfsr_bits, temp_lfsr_loc, sweep);
    } else {
        // Mark the data as wrong and keep going
        lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }

    // Undo the bit offset introduced above, to get the LFSR position of the first bit that hit the sensor.
    temp_lfsr_loc -= temp_bit_offset;

    // A count past a full rotation, or equal to the other sweep's, comes from a false polynomial match
    if (temp_lfsr_loc > db_lh2_period(basestation) / DB_LH2_PERIOD_TICKS_PER_COUNT) {
        lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }
    uint8_t other_sweep = sweep ^ 1;
    if (lh2->data_ready[other_sweep][basestation] == DB_LH2_PROCESSED_DATA_AVAILABLE && lh2->locations[other_sweep][basestation].lfsr_counts == temp_lfsr_loc) {
        lh2->data_ready[sweep][basestation]       = DB_LH2_NO_NEW_DATA;
        lh2->data_ready[other_sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }

    //*********************************************************************************//
    //                                 Store results                                   //
    //*********************************************************************************//

    // Save raw data information
    lh2->timestamps[sweep][basestation] = temp_timestamp;
    // Save processed location information
    lh2->locations[sweep][basestation].lfsr_counts         = temp_lfsr_loc;
    lh2->locations[sweep][basestation].selected_polynomial = temp_selected_polynomial;
    // Mark the data point as processed
    lh2->data_ready[sweep][basestation] = DB_LH2_PROCESSED_DATA_AVAILABLE;
}

void _update_lfsr_checkpoints(uint8_t polynomial, uint32_t bits, uint32_t count, uint8_t sweep) {

    // Save the new count in the correct place in the checkpoint array
    _lh2_vars.checkpoint.bits[polynomial][sweep]  = bits;
    _lh2_vars.checkpoint.count[polynomial][sweep] = count;
}

uint8_t _select_sweep(db_lh2_t *lh2, uint8_t polynomial, uint32_t timestamp) {
    // TODO: check the exact, per-mode period of each polynomial instead of using a blanket 20ms

    uint8_t  basestation  = polynomial >> 1;  ///< each base station uses 2 polynomials. integer dividing by 2 maps the polynomial number to the basestation number.
    uint16_t sweep_period = _lh2_sweep_period_us[basestation];
    uint32_t now          = db_timer_hf_now(LH2_TIMER_DEV);

    for (size_t sweep = 0; sweep < 2; sweep++) {
        if (now - lh2->timestamps[sweep][basestation] > LH2_MAX_DATA_VALID_TIME_US) {
            // Remove data that is too old.
            lh2->timestamps[sweep][basestation] = 0;
            lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
            // I don't think it's worth it to remove the location data. It is already marked as "No new data"
        }
    }

    ///< Encode in two bits which sweep slot of this basestation is empty, (1 means data, 0 means empty)
    uint8_t sweep_slot_state = ((lh2->timestamps[1][basestation] != 0) << 1) | (lh2->timestamps[0][basestation] != 0);
    // by default, select the first slot
    uint8_t selected_sweep = 0;

    switch (sweep_slot_state) {

        case LH2_SWEEP_BOTH_SLOTS_EMPTY:
        {
            // use the first slot
            selected_sweep = 0;
            break;
        }

        case LH2_SWEEP_FIRST_SLOT_EMPTY:
        {
            // check that the filled slot is not a perfect 20ms match to the new data.
            uint32_t diff = (timestamp - lh2->timestamps[1][basestation]) % sweep_period;
            diff          = diff < sweep_period - diff ? diff : sweep_period - diff;

            if (diff < LH2_SWEEP_PERIOD_THRESHOLD_US) {
                // match: use filled slot
                selected_sweep = 1;
            } else {
                // no match: use empty slot
                selected_sweep = 0;
            }
            break;
        }

        case LH2_SWEEP_SECOND_SLOT_EMPTY:
        {
            // check that the filled slot is not a perfect 20ms match to the new data.
            uint32_t diff = (timestamp - lh2->timestamps[0][basestation]) % sweep_period;
            diff          = diff < sweep_period - diff ? diff : sweep_period - diff;

            if (diff < LH2_SWEEP_PERIOD_THRESHOLD_US) {
                // match: use filled slot
                selected_sweep = 0;
            } else {
                // no match: use empty slot
                selected_sweep = 1;
            }
            break;
        }

        case LH2_SWEEP_BOTH_SLOTS_FULL:
        {
            // How far away is this new pulse from the already stored data
            int64_t diff_0 = ((timestamp - lh2->timestamps[0][basestation]) % sweep_period);
            diff_0         = diff_0 < sweep_period - diff_0 ? diff_0 : sweep_period - diff_0;
            int64_t diff_1 = ((timestamp - lh2->timestamps[1][basestation]) % sweep_period);
            diff_1         = diff_1 < sweep_period - diff_1 ? diff_1 : sweep_period - diff_1;

            // Use the one that is closest to 20ms
            if (diff_0 <= diff_1) {
                selected_sweep = 0;
            } else {
                selected_sweep = 1;
            }
            break;
        }

        default:
        {
            // By default, use he first slot
            selected_sweep = 0;
            break;
        }
    }

    return selected_sweep;
}

//=========================== drv/lh2_geometry/lh2_geometry.c =================

static const uint32_t _periods[DB_LH2_GEOMETRY_STATIONS] = {
    959000,
    957000,
    953000,
    949000,
    947000,
    943000,
    941000,
    939000,
    937000,
    929000,
    919000,
    911000,
    907000,
    901000,
    893000,
    887000,
};

uint32_t db_lh2_period(uint8_t station) {
    return (station < DB_LH2_GEOMETRY_STATIONS) ? _periods[station] : 0;
}

//=========================== hooks ============================================

/// In place of popping the ring's oldest capture, with the SPIM interrupt masked
static bool _get_from_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t *timestamp) {
    if (cb->count == 0) {
        return false;
    }
    memcpy(data, cb->buffer, SPI_BUFFER_SIZE);
    *timestamp = cb->timestamp;
    cb->count--;
    return true;
}

static uint32_t db_timer_hf_now(uint8_t dev) {
    (void)dev;
    return _now_us;
}

//=========================== private ==========================================

static bool _hash_table_ready = false;

static void _fill_hash_table(void) {
    for (uint8_t p = 0; p < LH2_POLYNOMIAL_COUNT; p++) {
        uint32_t last = 0;
        for (uint8_t i = 0; i < NUM_LSFR_COUNT_CHECKPOINTS; i++) {
            last = (_lfsr_index_table[p][i] > last) ? _lfsr_index_table[p][i] : last;
        }
        uint32_t state = 1;
        for (uint32_t n = 0; n <= last; n++) {
            for (uint8_t i = 0; i < NUM_LSFR_COUNT_CHECKPOINTS; i++) {
                if (_lfsr_index_table[p][i] == n) {
                    _lfsr_hash_table[p][i] = state;
                }
            }
            state = ((state << 1) | (__builtin_popcount(state & _polynomials[p]) & 1U)) & 0x1FFFFU;
        }
    }
    _hash_table_ready = true;
}

/// Which return the process function will take, from the same pure steps on
/// the state before the call; LH2_REF_STORED stands for "reaches the slot
/// checks", which the state after the call settles
static lh2_ref_result_t _classify(const uint8_t *spi, uint8_t *polynomial, uint32_t *count) {
    uint8_t temp_spi_bits[SPI_BUFFER_SIZE * 2] = { 0 };
    memcpy(temp_spi_bits, spi, SPI_BUFFER_SIZE);
    if (_check_mocap_interference(temp_spi_bits, SPI_BUFFER_SIZE)) {
        return LH2_REF_MOCAP;
    }
    uint64_t bits   = _demodulate_light(temp_spi_bits);
    int8_t   offset = 0;
    uint8_t  poly   = _determine_polynomial(bits, &offset);
    if (poly == LH2_POLYNOMIAL_ERROR_INDICATOR) {
        return LH2_REF_NO_POLYNOMIAL;
    }
    uint32_t lfsr_bits = bits >> (47 - offset);
    if (lfsr_bits == 0) {
        return LH2_REF_ZERO_BITS;
    }
    uint32_t loc = _lfsr_index_search(&_lh2_vars.checkpoint, poly, lfsr_bits);
    if (loc == LH2_LFSR_SEARCH_ERROR_INDICATOR) {
        return LH2_REF_NO_INDEX;
    }
    loc -= offset;
    if (loc > db_lh2_period(poly >> 1) / DB_LH2_PERIOD_TICKS_PER_COUNT) {
        return LH2_REF_PAST_REVOLUTION;
    }
    *polynomial = poly;
    *count      = loc;
    return LH2_REF_STORED;
}

static void _inconsistent(const char *what) {
    fprintf(stderr, "lh2_reference: the classification and the process function disagree: %s\n", what);
    abort();
}

//=========================== public ===========================================

void lh2_ref_init(lh2_ref_t *lh2) {
    if (!_hash_table_ready) {
        _fill_hash_table();
    }
    memset(&_lh2_vars, 0, sizeof(_lh2_vars));
    memset(lh2, 0, sizeof(*lh2));
    _now_us = 0;

    // db_lh2_init() at 228ce9a, past the hardware
    lh2->spi_ring_buffer_count_ptr = &_lh2_vars.data.count;
    for (uint8_t sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {
        for (uint8_t basestation = 0; basestation < LH2_BASESTATION_COUNT; basestation++) {
            lh2->locations[sweep][basestation].lfsr_counts         = LH2_LOCATION_ERROR_INDICATOR;
            lh2->locations[sweep][basestation].selected_polynomial = LH2_POLYNOMIAL_ERROR_INDICATOR;
            lh2->timestamps[sweep][basestation]                    = 0;
            lh2->data_ready[sweep][basestation]                    = DB_LH2_NO_NEW_DATA;
        }
    }
}

void lh2_ref_reset(lh2_ref_t *lh2) {
    // db_lh2_reset() at 228ce9a
    for (uint8_t basestation = 0; basestation < LH2_BASESTATION_COUNT; basestation++) {
        for (uint8_t sweep = 0; sweep < 2; sweep++) {
            lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
            lh2->timestamps[sweep][basestation] = 0;
        }
    }
    _lh2_vars.data.count = 0;
}

lh2_ref_result_t lh2_ref_feed(lh2_ref_t *lh2, const uint8_t spi[LH2_REF_CAPTURE_BYTES], uint32_t t_us, uint32_t now_us) {
    uint8_t          polynomial = 0;
    uint32_t         count      = 0;
    lh2_ref_result_t result     = _classify(spi, &polynomial, &count);
    lh2_ref_t        before;
    memcpy(&before, lh2, sizeof(before));

    memcpy(_lh2_vars.data.buffer, spi, SPI_BUFFER_SIZE);
    _lh2_vars.data.timestamp = t_us;
    _lh2_vars.data.count     = 1;
    _now_us                  = now_us;
    db_lh2_process_location(lh2);

    if (result == LH2_REF_MOCAP || result == LH2_REF_NO_POLYNOMIAL) {
        if (memcmp(&before, lh2, sizeof(before)) != 0) {
            _inconsistent("an early return changed the slots");
        }
        return result;
    }
    if (result != LH2_REF_STORED) {
        return result;
    }
    uint8_t station = polynomial >> 1;
    for (uint8_t sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {
        if (lh2->data_ready[sweep][station] == DB_LH2_PROCESSED_DATA_AVAILABLE && lh2->locations[sweep][station].lfsr_counts == count && lh2->locations[sweep][station].selected_polynomial == polynomial && lh2->timestamps[sweep][station] == t_us) {
            return LH2_REF_STORED;
        }
    }
    if (lh2->data_ready[0][station] != DB_LH2_NO_NEW_DATA || lh2->data_ready[1][station] != DB_LH2_NO_NEW_DATA) {
        _inconsistent("neither stored nor both slots cleared");
    }
    return LH2_REF_EQUAL_COUNTS;
}

const lh2_ref_checkpoint_t *lh2_ref_checkpoints(void) {
    return &_lh2_vars.checkpoint;
}

uint64_t lh2_ref_demodulate(const uint8_t spi[LH2_REF_CAPTURE_BYTES]) {
    uint8_t temp_spi_bits[SPI_BUFFER_SIZE * 2] = { 0 };
    memcpy(temp_spi_bits, spi, SPI_BUFFER_SIZE);
    return _demodulate_light(temp_spi_bits);
}

uint8_t lh2_ref_determine_polynomial(uint64_t bits, int8_t *start_val) {
    return _determine_polynomial(bits, start_val);
}

uint32_t lh2_ref_lfsr_index_search(lh2_ref_checkpoint_t *checkpoints, uint8_t polynomial, uint32_t bits) {
    if (!_hash_table_ready) {
        _fill_hash_table();
    }
    return _lfsr_index_search(checkpoints, polynomial, bits);
}

bool lh2_ref_mocap_interference(const uint8_t spi[LH2_REF_CAPTURE_BYTES]) {
    uint8_t copy[SPI_BUFFER_SIZE];
    memcpy(copy, spi, SPI_BUFFER_SIZE);
    return _check_mocap_interference(copy, SPI_BUFFER_SIZE);
}

uint32_t lh2_ref_polynomial(uint8_t index) {
    return (index < LH2_REF_POLYNOMIALS) ? _polynomials[index] : 0;
}

uint32_t lh2_ref_static_checkpoint(uint8_t polynomial, uint8_t slot) {
    return _lfsr_index_table[polynomial % LH2_REF_POLYNOMIALS][slot % LH2_REF_CHECKPOINTS];
}

uint32_t lh2_ref_static_checkpoint_state(uint8_t polynomial, uint8_t slot) {
    if (!_hash_table_ready) {
        _fill_hash_table();
    }
    return _lfsr_hash_table[polynomial % LH2_REF_POLYNOMIALS][slot % LH2_REF_CHECKPOINTS];
}
