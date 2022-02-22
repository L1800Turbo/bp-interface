/*
 * Si46xx_DAB_frequencies.c
 *
 *  Created on: Dec 30, 2021
 *      Author: kai
 */
#include "Si46xx_DAB_frequencies.h"

DAB_frequency_dt DAB_frequency_list[DAB_Chan_SIZE] = {
		{ .freq = 174928, .name = "5A"  },  /* DAB_Chan_5A  */
		{ .freq = 176640, .name = "5B"  },  /* DAB_Chan_5B  */
		{ .freq = 178352, .name = "5C"  },  /* DAB_Chan_5C  */
		{ .freq = 180064, .name = "5D"  },  /* DAB_Chan_5D  */
		{ .freq = 181936, .name = "6A"  },  /* DAB_Chan_6A  */
		{ .freq = 183648, .name = "6B"  },  /* DAB_Chan_6B  */
		{ .freq = 185360, .name = "6C"  },  /* DAB_Chan_6C  */
		{ .freq = 187072, .name = "6D"  },  /* DAB_Chan_6D  */
		{ .freq = 188928, .name = "7A"  },  /* DAB_Chan_7A  */
		{ .freq = 190640, .name = "7B"  },  /* DAB_Chan_7B  */
		{ .freq = 192352, .name = "7C"  },  /* DAB_Chan_7C  */
		{ .freq = 194064, .name = "7D"  },  /* DAB_Chan_7D  */
		{ .freq = 195936, .name = "8A"  },  /* DAB_Chan_8A  */
		{ .freq = 197648, .name = "8B"  },  /* DAB_Chan_8B  */
		{ .freq = 199360, .name = "8C"  },  /* DAB_Chan_8C  */
		{ .freq = 201072, .name = "8D"  },  /* DAB_Chan_8D  */
		{ .freq = 202928, .name = "9A"  },  /* DAB_Chan_9A  */
		{ .freq = 204640, .name = "9B"  },  /* DAB_Chan_9B  */
		{ .freq = 206352, .name = "9C"  },  /* DAB_Chan_9C  */
		{ .freq = 208064, .name = "9D"  },  /* DAB_Chan_9D  */
		{ .freq = 209936, .name = "10A" }, 	/* DAB_Chan_10A */
		{ .freq = 211648, .name = "10B" }, 	/* DAB_Chan_10B */
		{ .freq = 213360, .name = "10C" }, 	/* DAB_Chan_10C */
		{ .freq = 215072, .name = "10D" }, 	/* DAB_Chan_10D */
		{ .freq = 216928, .name = "11A" }, 	/* DAB_Chan_11A */
		{ .freq = 218640, .name = "11B" }, 	/* DAB_Chan_11B */
		{ .freq = 220352, .name = "11C" }, 	/* DAB_Chan_11C */
		{ .freq = 222064, .name = "11D" }, 	/* DAB_Chan_11D */
		{ .freq = 223936, .name = "12A" }, 	/* DAB_Chan_12A */
		{ .freq = 225648, .name = "12B" }, 	/* DAB_Chan_12B */
		{ .freq = 227360, .name = "12C" }, 	/* DAB_Chan_12C */
		{ .freq = 229072, .name = "12D" }, 	/* DAB_Chan_12D */
		{ .freq = 230784, .name = "13A" }, 	/* DAB_Chan_13A */
		{ .freq = 232496, .name = "13B" }, 	/* DAB_Chan_13B */
		{ .freq = 234208, .name = "13C" }, 	/* DAB_Chan_13C */
		{ .freq = 235776, .name = "13D" }, 	/* DAB_Chan_13D */
		{ .freq = 237488, .name = "13E" }, 	/* DAB_Chan_13E */
		{ .freq = 239200, .name = "13F" }  	/* DAB_Chan_13F */
};


