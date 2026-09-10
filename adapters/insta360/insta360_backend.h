/* SPDX-License-Identifier: MIT */

#ifndef INSTA360_BACKEND_H
#define INSTA360_BACKEND_H

#include "camera_backend.h"

/*
 * insta360_backend.h — insta360 后端（GATTS 从机）的 camera_backend 实现。
 */

/* 注册 insta360 后端到 camera_backend 注册表。 */
void insta360_backend_register(void);

#endif /* INSTA360_BACKEND_H */
