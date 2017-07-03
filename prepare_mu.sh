#!/bin/bash
rm mu.csv
rm mu2.csv
awk '!a[$0]++' std1/C_TINFO_CO_RE_UQ_MU_additional_log.csv std2/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu.csv
awk '!a[$0]++' mu.csv std3/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu2.csv
awk '!a[$0]++' mu2.csv std4/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu.csv
awk '!a[$0]++' mu.csv std5/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu2.csv
awk '!a[$0]++' mu2.csv std6/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu.csv
awk '!a[$0]++' mu.csv std7/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu2.csv
awk '!a[$0]++' mu2.csv std8/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu.csv
awk '!a[$0]++' mu.csv std9/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu2.csv
awk '!a[$0]++' mu2.csv std10/C_TINFO_CO_RE_UQ_MU_additional_log.csv > mu.csv
rm mu2.csv
