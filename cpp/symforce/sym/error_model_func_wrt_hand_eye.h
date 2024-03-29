// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: error_model_func_wrt_hand_eye
 *
 * Args:
 *     theta11: Scalar
 *     theta12: Scalar
 *     theta21: Scalar
 *     theta22: Scalar
 *     offset1: Scalar
 *     offset2: Scalar
 *     alpha: Scalar
 *     beta: Scalar
 *     RotGT1: Rot3
 *     RotGT2: Rot3
 *     RotHandEye: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix33
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> ErrorModelFuncWrtHandEye(
    const Scalar theta11, const Scalar theta12, const Scalar theta21, const Scalar theta22,
    const Scalar offset1, const Scalar offset2, const Scalar alpha, const Scalar beta,
    const sym::Rot3<Scalar>& RotGT1, const sym::Rot3<Scalar>& RotGT2,
    const sym::Rot3<Scalar>& RotHandEye, const Scalar epsilon) {
  // Total ops: 788

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _RotGT1 = RotGT1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _RotGT2 = RotGT2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _RotHandEye = RotHandEye.Data();

  // Intermediate terms (261)
  const Scalar _tmp0 = _RotGT1[0] * _RotGT2[0] + _RotGT1[1] * _RotGT2[1] + _RotGT1[2] * _RotGT2[2] +
                       _RotGT1[3] * _RotGT2[3];
  const Scalar _tmp1 = _RotHandEye[3] * _tmp0;
  const Scalar _tmp2 = -_RotGT1[0] * _RotGT2[3] - _RotGT1[1] * _RotGT2[2] +
                       _RotGT1[2] * _RotGT2[1] + _RotGT1[3] * _RotGT2[0];
  const Scalar _tmp3 = _RotHandEye[0] * _tmp2;
  const Scalar _tmp4 = _RotGT1[0] * _RotGT2[2] - _RotGT1[1] * _RotGT2[3] - _RotGT1[2] * _RotGT2[0] +
                       _RotGT1[3] * _RotGT2[1];
  const Scalar _tmp5 = _RotHandEye[1] * _tmp4;
  const Scalar _tmp6 = -_RotGT1[0] * _RotGT2[1] + _RotGT1[1] * _RotGT2[0] -
                       _RotGT1[2] * _RotGT2[3] + _RotGT1[3] * _RotGT2[2];
  const Scalar _tmp7 = _RotHandEye[2] * _tmp6;
  const Scalar _tmp8 = _tmp1 + _tmp3 + _tmp5 + _tmp7;
  const Scalar _tmp9 = _RotHandEye[2] * _tmp8;
  const Scalar _tmp10 = (Scalar(1) / Scalar(2)) * _tmp9;
  const Scalar _tmp11 = _RotHandEye[3] * _tmp2;
  const Scalar _tmp12 = (Scalar(1) / Scalar(2)) * _tmp11;
  const Scalar _tmp13 = _RotHandEye[2] * _tmp4;
  const Scalar _tmp14 = (Scalar(1) / Scalar(2)) * _tmp13;
  const Scalar _tmp15 = _RotHandEye[1] * _tmp6;
  const Scalar _tmp16 = (Scalar(1) / Scalar(2)) * _tmp15;
  const Scalar _tmp17 = _RotHandEye[0] * _tmp0;
  const Scalar _tmp18 = (Scalar(1) / Scalar(2)) * _tmp17;
  const Scalar _tmp19 = _tmp12 + _tmp14 - _tmp16 - _tmp18;
  const Scalar _tmp20 = _RotHandEye[1] * _tmp19;
  const Scalar _tmp21 = _RotHandEye[2] * _tmp2;
  const Scalar _tmp22 = (Scalar(1) / Scalar(2)) * _tmp21;
  const Scalar _tmp23 = _RotHandEye[3] * _tmp4;
  const Scalar _tmp24 = (Scalar(1) / Scalar(2)) * _tmp23;
  const Scalar _tmp25 = _RotHandEye[0] * _tmp6;
  const Scalar _tmp26 = (Scalar(1) / Scalar(2)) * _tmp25;
  const Scalar _tmp27 = _RotHandEye[1] * _tmp0;
  const Scalar _tmp28 = (Scalar(1) / Scalar(2)) * _tmp27;
  const Scalar _tmp29 = _tmp22 - _tmp24 - _tmp26 + _tmp28;
  const Scalar _tmp30 = _RotHandEye[1] * _tmp2;
  const Scalar _tmp31 = _RotHandEye[0] * _tmp4;
  const Scalar _tmp32 = _RotHandEye[3] * _tmp6;
  const Scalar _tmp33 = _RotHandEye[2] * _tmp0;
  const Scalar _tmp34 = _tmp30 - _tmp31 + _tmp32 - _tmp33;
  const Scalar _tmp35 = _RotHandEye[3] * _tmp34;
  const Scalar _tmp36 = (Scalar(1) / Scalar(2)) * _tmp35;
  const Scalar _tmp37 = _tmp11 + _tmp13 - _tmp15 - _tmp17;
  const Scalar _tmp38 = _RotHandEye[1] * _tmp37;
  const Scalar _tmp39 = (Scalar(1) / Scalar(2)) * _tmp38;
  const Scalar _tmp40 = -_tmp21 + _tmp23 + _tmp25 - _tmp27;
  const Scalar _tmp41 = _RotHandEye[0] * _tmp40;
  const Scalar _tmp42 = (Scalar(1) / Scalar(2)) * _tmp41;
  const Scalar _tmp43 = (Scalar(1) / Scalar(2)) * _tmp30;
  const Scalar _tmp44 = (Scalar(1) / Scalar(2)) * _tmp31;
  const Scalar _tmp45 = (Scalar(1) / Scalar(2)) * _tmp32;
  const Scalar _tmp46 = (Scalar(1) / Scalar(2)) * _tmp33;
  const Scalar _tmp47 = _tmp43 - _tmp44 + _tmp45 - _tmp46;
  const Scalar _tmp48 = -Scalar(1) / Scalar(2) * _tmp1 - Scalar(1) / Scalar(2) * _tmp3 -
                        Scalar(1) / Scalar(2) * _tmp5 - Scalar(1) / Scalar(2) * _tmp7;
  const Scalar _tmp49 = _RotHandEye[2] * _tmp48;
  const Scalar _tmp50 = _RotHandEye[3] * _tmp47 - _tmp49;
  const Scalar _tmp51 =
      _RotHandEye[0] * _tmp29 + _tmp10 + _tmp20 + _tmp36 + _tmp39 - _tmp42 + _tmp50;
  const Scalar _tmp52 = std::sin(alpha);
  const Scalar _tmp53 = offset1 + theta21;
  const Scalar _tmp54 = _tmp52 * std::cos(_tmp53);
  const Scalar _tmp55 = std::sin(beta);
  const Scalar _tmp56 = Scalar(1.0) / (_tmp55);
  const Scalar _tmp57 = std::cos(alpha);
  const Scalar _tmp58 = Scalar(1.0) / (_tmp57 + std::cos(beta));
  const Scalar _tmp59 = std::pow(_tmp55, Scalar(2));
  const Scalar _tmp60 = std::pow(_tmp52, Scalar(2));
  const Scalar _tmp61 = offset2 + theta22;
  const Scalar _tmp62 = (Scalar(1) / Scalar(2)) * _tmp53 - Scalar(1) / Scalar(2) * _tmp61;
  const Scalar _tmp63 =
      2 * std::atan(_tmp58 * (_tmp52 * std::cos(_tmp62) +
                              std::sqrt(Scalar(_tmp59 - _tmp60 * std::pow(Scalar(std::sin(_tmp62)),
                                                                          Scalar(2))))));
  const Scalar _tmp64 = std::cos(_tmp63);
  const Scalar _tmp65 = (Scalar(1) / Scalar(2)) * _tmp53 + (Scalar(1) / Scalar(2)) * _tmp61;
  const Scalar _tmp66 = std::cos(_tmp65);
  const Scalar _tmp67 = std::sin(_tmp63);
  const Scalar _tmp68 = _tmp57 * _tmp67;
  const Scalar _tmp69 = -_tmp54 * _tmp64 + _tmp66 * _tmp68;
  const Scalar _tmp70 = _tmp52 * std::sin(_tmp53);
  const Scalar _tmp71 = std::sin(_tmp65);
  const Scalar _tmp72 = _tmp64 * _tmp70 - _tmp68 * _tmp71;
  const Scalar _tmp73 = _tmp56 * (-_tmp54 * _tmp69 + _tmp70 * _tmp72);
  const Scalar _tmp74 = _tmp54 + _tmp73;
  const Scalar _tmp75 = _tmp54 * _tmp67 * _tmp71 - _tmp66 * _tmp67 * _tmp70;
  const Scalar _tmp76 = _tmp56 * (_tmp57 * _tmp69 - _tmp70 * _tmp75);
  const Scalar _tmp77 = _tmp56 * _tmp69;
  const Scalar _tmp78 = _tmp57 + _tmp76;
  const Scalar _tmp79 = _tmp77 + _tmp78;
  const Scalar _tmp80 =
      -std::max<Scalar>(_tmp57, std::max<Scalar>(_tmp76, std::max<Scalar>(_tmp77, _tmp79)));
  const Scalar _tmp81 =
      1 - std::max<Scalar>(0, -(((_tmp76 + _tmp80) > 0) - ((_tmp76 + _tmp80) < 0)));
  const Scalar _tmp82 = -_tmp77;
  const Scalar _tmp83 = 1 - _tmp57;
  const Scalar _tmp84 =
      -_tmp81 + std::sqrt(Scalar(std::max<Scalar>(0, _tmp76 + _tmp82 + _tmp83))) + 1;
  const Scalar _tmp85 = (Scalar(1) / Scalar(2)) * _tmp81;
  const Scalar _tmp86 = _tmp85 / _tmp84;
  const Scalar _tmp87 = _tmp56 * (_tmp54 * _tmp75 - _tmp57 * _tmp72);
  const Scalar _tmp88 = _tmp56 * _tmp72;
  const Scalar _tmp89 = -_tmp87 + _tmp88;
  const Scalar _tmp90 = _tmp77 + _tmp80;
  const Scalar _tmp91 = std::min<Scalar>(
      1 - std::max<Scalar>(0, _tmp81), 1 - std::max<Scalar>(0, -(((_tmp90) > 0) - ((_tmp90) < 0))));
  const Scalar _tmp92 = std::min<Scalar>(
      1 - std::max<Scalar>(0, std::max<Scalar>(_tmp81, _tmp91)),
      1 - std::max<Scalar>(0, -(((_tmp57 + _tmp80) > 0) - ((_tmp57 + _tmp80) < 0))));
  const Scalar _tmp93 = std::min<Scalar>(
      1 - std::max<Scalar>(0, std::max<Scalar>(_tmp81, std::max<Scalar>(_tmp91, _tmp92))),
      1 - std::max<Scalar>(0, -(((_tmp78 + _tmp90) > 0) - ((_tmp78 + _tmp90) < 0))));
  const Scalar _tmp94 = -_tmp93 + std::sqrt(Scalar(std::max<Scalar>(0, _tmp79 + 1))) + 1;
  const Scalar _tmp95 = (Scalar(1) / Scalar(2)) * _tmp93;
  const Scalar _tmp96 = _tmp95 / _tmp94;
  const Scalar _tmp97 = _tmp56 * _tmp75;
  const Scalar _tmp98 = _tmp70 + _tmp97;
  const Scalar _tmp99 = -_tmp76;
  const Scalar _tmp100 =
      -_tmp91 + std::sqrt(Scalar(std::max<Scalar>(0, _tmp77 + _tmp83 + _tmp99))) + 1;
  const Scalar _tmp101 = (Scalar(1) / Scalar(2)) * _tmp91;
  const Scalar _tmp102 = _tmp101 / _tmp100;
  const Scalar _tmp103 = _tmp57 + 1;
  const Scalar _tmp104 =
      -_tmp92 + std::sqrt(Scalar(std::max<Scalar>(0, _tmp103 + _tmp82 + _tmp99))) + 1;
  const Scalar _tmp105 = (Scalar(1) / Scalar(2)) * _tmp92;
  const Scalar _tmp106 = _tmp102 * _tmp98 + _tmp104 * _tmp105 + _tmp74 * _tmp86 + _tmp89 * _tmp96;
  const Scalar _tmp107 = offset2 + theta12;
  const Scalar _tmp108 = offset1 + theta11;
  const Scalar _tmp109 = -Scalar(1) / Scalar(2) * _tmp107 + (Scalar(1) / Scalar(2)) * _tmp108;
  const Scalar _tmp110 =
      2 * std::atan(_tmp58 * (_tmp52 * std::cos(_tmp109) +
                              std::sqrt(Scalar(_tmp59 - _tmp60 * std::pow(Scalar(std::sin(_tmp109)),
                                                                          Scalar(2))))));
  const Scalar _tmp111 = std::cos(_tmp110);
  const Scalar _tmp112 = _tmp52 * std::sin(_tmp108);
  const Scalar _tmp113 = (Scalar(1) / Scalar(2)) * _tmp107 + (Scalar(1) / Scalar(2)) * _tmp108;
  const Scalar _tmp114 = std::sin(_tmp113);
  const Scalar _tmp115 = std::sin(_tmp110);
  const Scalar _tmp116 = _tmp115 * _tmp57;
  const Scalar _tmp117 = _tmp111 * _tmp112 - _tmp114 * _tmp116;
  const Scalar _tmp118 = std::cos(_tmp113);
  const Scalar _tmp119 = _tmp52 * std::cos(_tmp108);
  const Scalar _tmp120 = -_tmp112 * _tmp115 * _tmp118 + _tmp114 * _tmp115 * _tmp119;
  const Scalar _tmp121 = _tmp56 * (-_tmp117 * _tmp57 + _tmp119 * _tmp120);
  const Scalar _tmp122 = _tmp117 * _tmp56;
  const Scalar _tmp123 = _tmp121 + _tmp122;
  const Scalar _tmp124 = -_tmp111 * _tmp119 + _tmp116 * _tmp118;
  const Scalar _tmp125 = _tmp56 * (-_tmp112 * _tmp120 + _tmp124 * _tmp57);
  const Scalar _tmp126 = _tmp124 * _tmp56;
  const Scalar _tmp127 = _tmp125 + _tmp126;
  const Scalar _tmp128 = _tmp127 + _tmp57;
  const Scalar _tmp129 =
      -std::max<Scalar>(_tmp125, std::max<Scalar>(_tmp126, std::max<Scalar>(_tmp128, _tmp57)));
  const Scalar _tmp130 =
      1 - std::max<Scalar>(0, -(((_tmp125 + _tmp129) > 0) - ((_tmp125 + _tmp129) < 0)));
  const Scalar _tmp131 = std::min<Scalar>(
      1 - std::max<Scalar>(0, _tmp130),
      1 - std::max<Scalar>(0, -(((_tmp126 + _tmp129) > 0) - ((_tmp126 + _tmp129) < 0))));
  const Scalar _tmp132 = -_tmp125;
  const Scalar _tmp133 =
      -_tmp131 + std::sqrt(Scalar(std::max<Scalar>(0, _tmp126 + _tmp132 + _tmp83))) + 1;
  const Scalar _tmp134 = (Scalar(1) / Scalar(2)) * _tmp131;
  const Scalar _tmp135 = _tmp134 / _tmp133;
  const Scalar _tmp136 = -_tmp126;
  const Scalar _tmp137 =
      -_tmp130 + std::sqrt(Scalar(std::max<Scalar>(0, _tmp125 + _tmp136 + _tmp83))) + 1;
  const Scalar _tmp138 = (Scalar(1) / Scalar(2)) * _tmp130;
  const Scalar _tmp139 = _tmp56 * (_tmp112 * _tmp117 - _tmp119 * _tmp124);
  const Scalar _tmp140 = _tmp119 + _tmp139;
  const Scalar _tmp141 = _tmp129 + _tmp57;
  const Scalar _tmp142 =
      std::min<Scalar>(1 - std::max<Scalar>(0, std::max<Scalar>(_tmp130, _tmp131)),
                       1 - std::max<Scalar>(0, -(((_tmp141) > 0) - ((_tmp141) < 0))));
  const Scalar _tmp143 =
      -_tmp142 + std::sqrt(Scalar(std::max<Scalar>(0, _tmp103 + _tmp132 + _tmp136))) + 1;
  const Scalar _tmp144 = (Scalar(1) / Scalar(2)) * _tmp142;
  const Scalar _tmp145 = _tmp144 / _tmp143;
  const Scalar _tmp146 = _tmp120 * _tmp56;
  const Scalar _tmp147 = _tmp112 - _tmp146;
  const Scalar _tmp148 = std::min<Scalar>(
      1 - std::max<Scalar>(0, std::max<Scalar>(_tmp130, std::max<Scalar>(_tmp131, _tmp142))),
      1 - std::max<Scalar>(0, -(((_tmp127 + _tmp141) > 0) - ((_tmp127 + _tmp141) < 0))));
  const Scalar _tmp149 = -_tmp148 + std::sqrt(Scalar(std::max<Scalar>(0, _tmp128 + 1))) + 1;
  const Scalar _tmp150 = (Scalar(1) / Scalar(2)) * _tmp148;
  const Scalar _tmp151 = _tmp150 / _tmp149;
  const Scalar _tmp152 =
      _tmp123 * _tmp135 + _tmp137 * _tmp138 + _tmp140 * _tmp145 + _tmp147 * _tmp151;
  const Scalar _tmp153 = _tmp87 + _tmp88;
  const Scalar _tmp154 = _tmp70 - _tmp97;
  const Scalar _tmp155 = _tmp105 / _tmp104;
  const Scalar _tmp156 = _tmp102 * _tmp153 + _tmp154 * _tmp96 + _tmp155 * _tmp74 + _tmp84 * _tmp85;
  const Scalar _tmp157 = _tmp112 + _tmp146;
  const Scalar _tmp158 = _tmp138 / _tmp137;
  const Scalar _tmp159 = -_tmp121 + _tmp122;
  const Scalar _tmp160 =
      _tmp135 * _tmp157 + _tmp140 * _tmp158 + _tmp143 * _tmp144 + _tmp151 * _tmp159;
  const Scalar _tmp161 = -_tmp119 + _tmp139;
  const Scalar _tmp162 =
      _tmp123 * _tmp158 + _tmp133 * _tmp134 + _tmp145 * _tmp157 + _tmp151 * _tmp161;
  const Scalar _tmp163 = -_tmp54 + _tmp73;
  const Scalar _tmp164 = _tmp102 * _tmp163 + _tmp154 * _tmp86 + _tmp155 * _tmp89 + _tmp94 * _tmp95;
  const Scalar _tmp165 =
      _tmp135 * _tmp161 + _tmp145 * _tmp159 + _tmp147 * _tmp158 + _tmp149 * _tmp150;
  const Scalar _tmp166 = _tmp100 * _tmp101 + _tmp153 * _tmp86 + _tmp155 * _tmp98 + _tmp163 * _tmp96;
  const Scalar _tmp167 =
      _tmp106 * _tmp152 - _tmp156 * _tmp160 - _tmp162 * _tmp164 + _tmp165 * _tmp166;
  const Scalar _tmp168 = _RotHandEye[1] * _tmp47;
  const Scalar _tmp169 = _RotHandEye[0] * _tmp48;
  const Scalar _tmp170 = _RotHandEye[3] * _tmp19 - _tmp169;
  const Scalar _tmp171 = _RotHandEye[0] * _tmp8;
  const Scalar _tmp172 = (Scalar(1) / Scalar(2)) * _tmp171;
  const Scalar _tmp173 = _RotHandEye[1] * _tmp34;
  const Scalar _tmp174 = (Scalar(1) / Scalar(2)) * _tmp173;
  const Scalar _tmp175 = _RotHandEye[3] * _tmp37;
  const Scalar _tmp176 = (Scalar(1) / Scalar(2)) * _tmp175;
  const Scalar _tmp177 = _RotHandEye[2] * _tmp40;
  const Scalar _tmp178 = (Scalar(1) / Scalar(2)) * _tmp177;
  const Scalar _tmp179 = -_tmp172 + _tmp174 - _tmp176 - _tmp178;
  const Scalar _tmp180 = -_RotHandEye[2] * _tmp29 - _tmp168 + _tmp170 + _tmp179;
  const Scalar _tmp181 =
      _tmp106 * _tmp160 + _tmp152 * _tmp156 + _tmp162 * _tmp166 + _tmp164 * _tmp165;
  const Scalar _tmp182 = _RotHandEye[3] * _tmp8;
  const Scalar _tmp183 = (Scalar(1) / Scalar(2)) * _tmp182;
  const Scalar _tmp184 = _RotHandEye[2] * _tmp47;
  const Scalar _tmp185 = _RotHandEye[3] * _tmp48;
  const Scalar _tmp186 = _RotHandEye[0] * _tmp19;
  const Scalar _tmp187 = _RotHandEye[2] * _tmp34;
  const Scalar _tmp188 = -Scalar(1) / Scalar(2) * _tmp187;
  const Scalar _tmp189 = _RotHandEye[0] * _tmp37;
  const Scalar _tmp190 = -Scalar(1) / Scalar(2) * _tmp189;
  const Scalar _tmp191 = _RotHandEye[1] * _tmp40;
  const Scalar _tmp192 = -Scalar(1) / Scalar(2) * _tmp191;
  const Scalar _tmp193 = -_RotHandEye[1] * _tmp29 + _tmp183 + _tmp184 + _tmp185 + _tmp186 +
                         _tmp188 + _tmp190 + _tmp192;
  const Scalar _tmp194 =
      -_tmp106 * _tmp162 - _tmp152 * _tmp164 + _tmp156 * _tmp165 + _tmp160 * _tmp166;
  const Scalar _tmp195 = _RotHandEye[0] * _tmp47;
  const Scalar _tmp196 = _RotHandEye[1] * _tmp48;
  const Scalar _tmp197 = _RotHandEye[2] * _tmp19;
  const Scalar _tmp198 = _RotHandEye[1] * _tmp8;
  const Scalar _tmp199 = (Scalar(1) / Scalar(2)) * _tmp198;
  const Scalar _tmp200 = _RotHandEye[0] * _tmp34;
  const Scalar _tmp201 = (Scalar(1) / Scalar(2)) * _tmp200;
  const Scalar _tmp202 = _RotHandEye[2] * _tmp37;
  const Scalar _tmp203 = (Scalar(1) / Scalar(2)) * _tmp202;
  const Scalar _tmp204 = _RotHandEye[3] * _tmp40;
  const Scalar _tmp205 = (Scalar(1) / Scalar(2)) * _tmp204;
  const Scalar _tmp206 = -_tmp199 - _tmp201 + _tmp203 - _tmp205;
  const Scalar _tmp207 = _RotHandEye[3] * _tmp29 - _tmp195 + _tmp196 + _tmp197 + _tmp206;
  const Scalar _tmp208 =
      _tmp106 * _tmp165 - _tmp152 * _tmp166 + _tmp156 * _tmp162 - _tmp160 * _tmp164;
  const Scalar _tmp209 =
      _tmp167 * _tmp51 + _tmp180 * _tmp181 + _tmp193 * _tmp194 + _tmp207 * _tmp208;
  const Scalar _tmp210 = 1 - epsilon;
  const Scalar _tmp211 = _tmp171 - _tmp173 + _tmp175 + _tmp177;
  const Scalar _tmp212 = _tmp194 * _tmp211;
  const Scalar _tmp213 = _tmp182 - _tmp187 - _tmp189 - _tmp191;
  const Scalar _tmp214 = _tmp181 * _tmp213;
  const Scalar _tmp215 = _tmp35 + _tmp38 - _tmp41 + _tmp9;
  const Scalar _tmp216 = _tmp208 * _tmp215;
  const Scalar _tmp217 = _tmp198 + _tmp200 - _tmp202 + _tmp204;
  const Scalar _tmp218 = _tmp167 * _tmp217;
  const Scalar _tmp219 = _tmp212 + _tmp214 + _tmp216 + _tmp218;
  const Scalar _tmp220 = std::fabs(_tmp219);
  const Scalar _tmp221 = std::min<Scalar>(_tmp210, _tmp220);
  const Scalar _tmp222 = 1 - std::pow(_tmp221, Scalar(2));
  const Scalar _tmp223 = 2 * std::min<Scalar>(0, (((_tmp212 + _tmp214 + _tmp216 + _tmp218) > 0) -
                                                  ((_tmp212 + _tmp214 + _tmp216 + _tmp218) < 0))) +
                         1;
  const Scalar _tmp224 = _tmp223 * std::acos(_tmp221);
  const Scalar _tmp225 = _tmp221 * _tmp224 / (_tmp222 * std::sqrt(_tmp222));
  const Scalar _tmp226 = _tmp209 * _tmp225;
  const Scalar _tmp227 =
      _tmp167 * _tmp215 - _tmp181 * _tmp211 + _tmp194 * _tmp213 - _tmp208 * _tmp217;
  const Scalar _tmp228 = ((((_tmp210 - _tmp220) > 0) - ((_tmp210 - _tmp220) < 0)) + 1) *
                         (((_tmp219) > 0) - ((_tmp219) < 0));
  const Scalar _tmp229 = _tmp227 * _tmp228;
  const Scalar _tmp230 = _tmp223 / _tmp222;
  const Scalar _tmp231 = _tmp228 * _tmp230;
  const Scalar _tmp232 = _tmp227 * _tmp231;
  const Scalar _tmp233 = 2 * _tmp224 / std::sqrt(_tmp222);
  const Scalar _tmp234 =
      _tmp167 * _tmp213 - _tmp181 * _tmp217 - _tmp194 * _tmp215 + _tmp208 * _tmp211;
  const Scalar _tmp235 = _tmp231 * _tmp234;
  const Scalar _tmp236 = _tmp228 * _tmp234;
  const Scalar _tmp237 =
      _tmp228 * (-_tmp167 * _tmp211 - _tmp181 * _tmp215 + _tmp194 * _tmp217 + _tmp208 * _tmp213);
  const Scalar _tmp238 = _tmp209 * _tmp237;
  const Scalar _tmp239 = -_tmp43 + _tmp44 - _tmp45 + _tmp46;
  const Scalar _tmp240 = -_tmp22 + _tmp24 + _tmp26 - _tmp28;
  const Scalar _tmp241 = _RotHandEye[1] * _tmp240 + _tmp183 + _tmp185 + _tmp188 + _tmp190 + _tmp192;
  const Scalar _tmp242 = -_RotHandEye[2] * _tmp239 + _tmp186 + _tmp241;
  const Scalar _tmp243 = _RotHandEye[3] * _tmp240 - _tmp196;
  const Scalar _tmp244 = -_RotHandEye[0] * _tmp239 - _tmp197 + _tmp206 + _tmp243;
  const Scalar _tmp245 = _RotHandEye[0] * _tmp240;
  const Scalar _tmp246 = -_tmp10 - _tmp36 - _tmp39 + _tmp42;
  const Scalar _tmp247 = _RotHandEye[3] * _tmp239 - _tmp20 + _tmp245 + _tmp246 + _tmp49;
  const Scalar _tmp248 = _RotHandEye[2] * _tmp240;
  const Scalar _tmp249 =
      _RotHandEye[1] * _tmp239 + _tmp170 + _tmp172 - _tmp174 + _tmp176 + _tmp178 + _tmp248;
  const Scalar _tmp250 =
      _tmp167 * _tmp242 + _tmp181 * _tmp244 + _tmp194 * _tmp247 + _tmp208 * _tmp249;
  const Scalar _tmp251 = _tmp225 * _tmp250;
  const Scalar _tmp252 = _tmp230 * _tmp237;
  const Scalar _tmp253 = -_tmp12 - _tmp14 + _tmp16 + _tmp18;
  const Scalar _tmp254 = _RotHandEye[3] * _tmp253 + _tmp168 + _tmp169 + _tmp179 - _tmp248;
  const Scalar _tmp255 = -_RotHandEye[1] * _tmp253 - _tmp245 + _tmp246 + _tmp50;
  const Scalar _tmp256 =
      _RotHandEye[2] * _tmp253 + _tmp195 + _tmp199 + _tmp201 - _tmp203 + _tmp205 + _tmp243;
  const Scalar _tmp257 = -_RotHandEye[0] * _tmp253 + _tmp184 + _tmp241;
  const Scalar _tmp258 =
      _tmp167 * _tmp254 + _tmp181 * _tmp255 + _tmp194 * _tmp256 + _tmp208 * _tmp257;
  const Scalar _tmp259 = _tmp225 * _tmp258;
  const Scalar _tmp260 = _tmp228 * _tmp259;

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 3> _res;

  _res(0, 0) =
      -_tmp209 * _tmp232 + _tmp226 * _tmp229 +
      _tmp233 * (_tmp167 * _tmp207 + _tmp180 * _tmp194 - _tmp181 * _tmp193 - _tmp208 * _tmp51);
  _res(1, 0) =
      -_tmp209 * _tmp235 + _tmp226 * _tmp236 +
      _tmp233 * (_tmp167 * _tmp180 - _tmp181 * _tmp51 + _tmp193 * _tmp208 - _tmp194 * _tmp207);
  _res(2, 0) =
      _tmp225 * _tmp238 - _tmp230 * _tmp238 +
      _tmp233 * (-_tmp167 * _tmp193 + _tmp180 * _tmp208 - _tmp181 * _tmp207 + _tmp194 * _tmp51);
  _res(0, 1) =
      _tmp229 * _tmp251 - _tmp232 * _tmp250 +
      _tmp233 * (_tmp167 * _tmp249 - _tmp181 * _tmp247 + _tmp194 * _tmp244 - _tmp208 * _tmp242);
  _res(1, 1) =
      _tmp233 * (_tmp167 * _tmp244 - _tmp181 * _tmp242 - _tmp194 * _tmp249 + _tmp208 * _tmp247) -
      _tmp235 * _tmp250 + _tmp236 * _tmp251;
  _res(2, 1) =
      _tmp233 * (-_tmp167 * _tmp247 - _tmp181 * _tmp249 + _tmp194 * _tmp242 + _tmp208 * _tmp244) +
      _tmp237 * _tmp251 - _tmp250 * _tmp252;
  _res(0, 2) =
      _tmp227 * _tmp260 - _tmp232 * _tmp258 +
      _tmp233 * (_tmp167 * _tmp257 - _tmp181 * _tmp256 + _tmp194 * _tmp255 - _tmp208 * _tmp254);
  _res(1, 2) =
      _tmp233 * (_tmp167 * _tmp255 - _tmp181 * _tmp254 - _tmp194 * _tmp257 + _tmp208 * _tmp256) +
      _tmp234 * _tmp260 - _tmp235 * _tmp258;
  _res(2, 2) =
      _tmp233 * (-_tmp167 * _tmp256 - _tmp181 * _tmp257 + _tmp194 * _tmp254 + _tmp208 * _tmp255) +
      _tmp237 * _tmp259 - _tmp252 * _tmp258;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
