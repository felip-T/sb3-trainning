build_lib_example(
  NAME modelTeste
  SOURCE_FILES teste.cc train.cc open-gym.cc oran-report-sinr-enb.cc oran-report-sinr-ue.cc enbReporterAvgSinr.cc
  LIBRARIES_TO_LINK ${libcore} ${liblte} ${libmobility} ${libbuilds} ${liboran} ${libai} ${libflow-monitor}
  )
