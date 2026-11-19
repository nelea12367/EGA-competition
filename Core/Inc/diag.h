/**
 * @file diag.h
 * @brief 硬件诊断模式：测"遥控信号到底进没进板子、进到哪个脚"。
 */

#ifndef DIAG_H
#define DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

/** 进入诊断循环（不返回）。结果放在 diag.c 的全局变量里，用 SWD 读。 */
void Diag_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_H */
