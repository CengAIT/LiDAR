#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define MAX_LINE 512
#define MAX_POINTS 2000
#define MAX_LINES 50
#define MLESAC_ITERATIONS 2000
#define DIST_THRESHOLD 0.05
#define MLESAC_SIGMA 0.05
#define MIN_INLIERS 8
#define MIN_CORNER_ANGLE 60.0
#define WINDOW_WIDTH 1600
#define WINDOW_HEIGHT 1000
#define SCALE 70.0
#define ORIGIN_X (WINDOW_WIDTH / 2)
#define ORIGIN_Y (WINDOW_HEIGHT / 2)

typedef struct { char stamp[64]; char frame_id[64]; } Header;
typedef struct { float angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max, ranges[2000]; int range_count; } Scan;
typedef struct { Header header; Scan scan; } LidarData;
typedef struct { double x, y; int idx; } Point2D;
typedef struct { double a, b, c; int inlier_count; double score; int *inlier_indices; } LineModel;
typedef struct { Point2D start, end; LineModel model; } Line;
typedef struct { Point2D intersection_point; float angle_deg, distance_from_robot; int lineA, lineB; } ValidIntersection;

void delete_space(char *data) {
    if (!data) return;
    char *start = data;
    while (isspace((unsigned char)*start)) start++;
    if (start != data) memmove(data, start, strlen(start)+1);
    char *end = data + strlen(data)-1;
    while (end >= data && isspace((unsigned char)*end)) { *end = '\0'; end--; }
}

int load_lidar_data(const char *filename, LidarData *lidar) {
    FILE *toml = fopen(filename, "r");
    if (!toml) { printf("Dosya acilamadi: %s\n", filename); return 0; }
    memset(lidar, 0, sizeof(LidarData));
    char line[MAX_LINE], section[32] = "";
    int reading_ranges = 0;
    while (fgets(line, sizeof(line), toml)) {
        delete_space(line);
        if (strlen(line) == 0 || line[0] == '#') continue;
        if (line[0] == '[') { sscanf(line, "[%[^]]", section); continue; }
        if (strncmp(line, "ranges", 6) == 0) {
            reading_ranges = 1; lidar->scan.range_count = 0;
            char *br = strchr(line, '[');
            if (br) {
                char *p = br+1, *token = strtok(p, ",");
                while (token) {
                    delete_space(token);
                    char *rb = strchr(token, ']');
                    if (rb) *rb = '\0';
                    if (strlen(token) > 0) lidar->scan.ranges[lidar->scan.range_count++] = atof(token);
                    token = strtok(NULL, ",");
                }
                if (strchr(line, ']')) reading_ranges = 0;
            }
            continue;
        }
        if (reading_ranges) {
            char *p = line, *rb = strchr(p, ']');
            if (rb) *rb = '\0';
            char *token = strtok(p, ",");
            while (token) {
                delete_space(token);
                if (strlen(token) > 0) lidar->scan.ranges[lidar->scan.range_count++] = atof(token);
                token = strtok(NULL, ",");
            }
            if (rb) reading_ranges = 0;
            continue;
        }
        char key[64], value[128];
        if (sscanf(line, "%[^=]=%[^\n]", key, value) == 2) {
            delete_space(key); delete_space(value);
            if (value[0] == '"' && value[strlen(value)-1] == '"') {
                value[strlen(value)-1] = '\0';
                memmove(value, value+1, strlen(value)+1);
            }
            if (strcmp(section, "header") == 0) {
                if (strcmp(key, "stamp") == 0) strncpy(lidar->header.stamp, value, 63);
                else if (strcmp(key, "frame_id") == 0) strncpy(lidar->header.frame_id, value, 63);
            } else if (strcmp(section, "scan") == 0) {
                if (strcmp(key, "angle_min") == 0) lidar->scan.angle_min = atof(value);
                else if (strcmp(key, "angle_max") == 0) lidar->scan.angle_max = atof(value);
                else if (strcmp(key, "angle_increment") == 0) lidar->scan.angle_increment = atof(value);
                else if (strcmp(key, "range_min") == 0) lidar->scan.range_min = atof(value);
                else if (strcmp(key, "range_max") == 0) lidar->scan.range_max = atof(value);
            }
        }
    }
    fclose(toml); return 1;
}

int filter_and_convert_ranges(LidarData *lidar, Point2D *points) {
    int valid_count = 0;
    for (int i = 0; i < lidar->scan.range_count && i < MAX_POINTS; i++) {
        float r = lidar->scan.ranges[i];
        if (!(r > 0.0f) || r < lidar->scan.range_min || r > lidar->scan.range_max) continue;
        double angle = lidar->scan.angle_min + i*lidar->scan.angle_increment;
        points[valid_count].x = r*cos(angle);
        points[valid_count].y = r*sin(angle);
        points[valid_count++].idx = i;
    }
    return valid_count;
}

int line_from_two_points(Point2D p1, Point2D p2, LineModel *m) {
    double dx = p2.x-p1.x, dy = p2.y-p1.y;
    if (fabs(dx) < 1e-8 && fabs(dy) < 1e-8) return 0;
    double a = -dy, b = dx, norm = sqrt(a*a + b*b);
    if (norm < 1e-12) return 0;
    m->a = a/norm; m->b = b/norm; m->c = -(m->a*p1.x + m->b*p1.y);
    m->inlier_count = 0; m->score = 0.0; m->inlier_indices = NULL;
    return 1;
}

double point_line_distance(const LineModel *m, const Point2D *p) {
    return fabs(m->a*p->x + m->b*p->y + m->c);
}

int refine_line_least_squares(Point2D pts[], int npts, LineModel *out) {
    if (npts < 2) return 0;
    double meanx = 0, meany = 0;
    for (int i = 0; i < npts; i++) { meanx += pts[i].x; meany += pts[i].y; }
    meanx /= npts; meany /= npts;
    double Sxx = 0, Sxy = 0, Syy = 0;
    for (int i = 0; i < npts; i++) {
        double dx = pts[i].x-meanx, dy = pts[i].y-meany;
        Sxx += dx*dx; Sxy += dx*dy; Syy += dy*dy;
    }
    double trace = Sxx+Syy, det = Sxx*Syy-Sxy*Sxy, tmp = trace*trace-4*det;
    if (tmp < 0) tmp = 0;
    double lambda = 0.5*(trace + sqrt(tmp)), vx = Sxy, vy = lambda-Sxx;
    if (fabs(vx) < 1e-8 && fabs(vy) < 1e-8) { vx = 1.0; vy = 0.0; }
    double na = -vy, nb = vx, norm = sqrt(na*na + nb*nb);
    if (norm < 1e-12) return 0;
    out->a = na/norm; out->b = nb/norm; out->c = -(out->a*meanx + out->b*meany);
    return 1;
}

double compute_mlesac_score(const LineModel *m, Point2D pts[], int npts, double sigma) {
    double s = 0.0, var2 = 2.0*sigma*sigma;
    for (int i = 0; i < npts; i++) {
        double d = point_line_distance(m, &pts[i]);
        s += exp(-(d*d)/var2);
    }
    return s;
}

void rand_two_indices(int n, int *i1, int *i2) {
    *i1 = rand() % n;
    do { *i2 = rand() % n; } while (*i2 == *i1);
}

int mlesac_find_best_line(Point2D pts[], int npts, LineModel *best_model, int *best_inliers_idx, int *best_inliers_count) {
    if (npts < 2) return 0;
    double best_score = -1e300;
    int best_count = 0, *tmp_inliers = malloc(sizeof(int)*npts);
    if (!tmp_inliers) return 0;
    for (int it = 0; it < MLESAC_ITERATIONS; it++) {
        int i1, i2;
        rand_two_indices(npts, &i1, &i2);
        LineModel candidate;
        if (!line_from_two_points(pts[i1], pts[i2], &candidate)) continue;
        double score = compute_mlesac_score(&candidate, pts, npts, MLESAC_SIGMA);
        int count = 0;
        for (int i = 0; i < npts; i++) {
            if (point_line_distance(&candidate, &pts[i]) < DIST_THRESHOLD) tmp_inliers[count++] = i;
        }
        if (score > best_score || (fabs(score-best_score) < 1e-9 && count > best_count)) {
            best_score = score; best_count = count; *best_inliers_count = count;
            candidate.score = score; candidate.inlier_count = count; *best_model = candidate;
            for (int k = 0; k < count; k++) best_inliers_idx[k] = tmp_inliers[k];
        }
    }
    free(tmp_inliers);
    return (best_count >= MIN_INLIERS) ? 1 : 0;
}

int extract_all_lines(Point2D pts[], int *npts, Line *lines, int max_lines) {
    int remaining = *npts;
    Point2D *working = malloc(sizeof(Point2D)*remaining);
    if (!working) return 0;
    for (int i = 0; i < remaining; i++) working[i] = pts[i];
    int line_count = 0;
    while (line_count < max_lines && remaining >= MIN_INLIERS) {
        LineModel best;
        int *inliers_idx = malloc(sizeof(int)*remaining), inlier_count = 0;
        if (!inliers_idx) break;
        if (!mlesac_find_best_line(working, remaining, &best, inliers_idx, &inlier_count)) { free(inliers_idx); break; }
        Point2D *inliers_pts = malloc(sizeof(Point2D)*inlier_count);
        if (!inliers_pts) { free(inliers_idx); break; }
        for (int k = 0; k < inlier_count; k++) inliers_pts[k] = working[inliers_idx[k]];
        refine_line_least_squares(inliers_pts, inlier_count, &best);
        int final_cnt = 0, *final_idx = malloc(sizeof(int)*remaining);
        if (!final_idx) { free(inliers_pts); free(inliers_idx); break; }
        for (int i = 0; i < remaining; i++)
            if (point_line_distance(&best, &working[i]) < DIST_THRESHOLD) final_idx[final_cnt++] = working[i].idx;
        if (final_cnt < MIN_INLIERS) { free(inliers_pts); free(inliers_idx); free(final_idx); break; }
        double max_dist = 0; int idx1 = 0, idx2 = 0;
        for (int j = 0; j < inlier_count; j++) {
            for (int k = j+1; k < inlier_count; k++) {
                double dx = inliers_pts[k].x-inliers_pts[j].x, dy = inliers_pts[k].y-inliers_pts[j].y;
                double dist = sqrt(dx*dx + dy*dy);
                if (dist > max_dist) { max_dist = dist; idx1 = j; idx2 = k; }
            }
        }
        lines[line_count].start = inliers_pts[idx1]; lines[line_count].end = inliers_pts[idx2];
        lines[line_count].model = best; lines[line_count].model.inlier_count = final_cnt;
        lines[line_count++].model.inlier_indices = final_idx;
        int *is_inlier = calloc(remaining, sizeof(int));
        if (!is_inlier) { free(inliers_pts); free(inliers_idx); break; }
        for (int k = 0; k < inlier_count; k++) { int idx = inliers_idx[k]; if (idx >= 0 && idx < remaining) is_inlier[idx] = 1; }
        Point2D *newworking = malloc(sizeof(Point2D)*remaining);
        if (!newworking) { free(is_inlier); free(inliers_pts); free(inliers_idx); break; }
        int newcount = 0;
        for (int i = 0; i < remaining; i++) if (!is_inlier[i]) newworking[newcount++] = working[i];
        free(is_inlier); free(working); free(inliers_pts); free(inliers_idx);
        working = newworking; remaining = newcount;
    }
    *npts = remaining; free(working);
    return line_count;
}

int isLineIntersecting(Line a, Line b, Point2D *intersection) {
    double x1=a.start.x, y1=a.start.y, x2=a.end.x, y2=a.end.y, x3=b.start.x, y3=b.start.y, x4=b.end.x, y4=b.end.y;
    double denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if (fabs(denom) < 1e-9) return 0;
    double px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4))/denom;
    double py = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4))/denom;
    double t_a = (fabs(x2-x1)>1e-9) ? (px-x1)/(x2-x1) : (py-y1)/(y2-y1);
    double t_b = (fabs(x4-x3)>1e-9) ? (px-x3)/(x4-x3) : (py-y3)/(y4-y3);
    if (t_a < 0 || t_a > 1 || t_b < 0 || t_b > 1) return 0;
    intersection->x = px; intersection->y = py;
    return 1;
}

double calculateAngleBetweenLines(Line a, Line b) {
    double dx1=a.end.x-a.start.x, dy1=a.end.y-a.start.y, dx2=b.end.x-b.start.x, dy2=b.end.y-b.start.y;
    return atan2(fabs(dx1*dy2 - dy1*dx2), dx1*dx2 + dy1*dy2);
}

void world_to_screen(double wx, double wy, int *sx, int *sy) {
    *sx = ORIGIN_X + (int)(wx*SCALE);
    *sy = ORIGIN_Y - (int)(wy*SCALE);
}

void draw_grid(SDL_Renderer *r, TTF_Font *font) {
    SDL_SetRenderDrawColor(r, 255, 255, 255, 255);  // Tamamen beyaz
    SDL_RenderClear(r);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(r, 230, 230, 230, 255);  // Cok hafif gri izgara
    int step_count_x = 15, step_count_y = 10;
    for (int i = -step_count_x; i <= step_count_x; i++) {
        int x = ORIGIN_X + i*(int)SCALE;
        if (x >= 0 && x <= WINDOW_WIDTH) SDL_RenderDrawLine(r, x, 0, x, WINDOW_HEIGHT);
    }
    for (int i = -step_count_y; i <= step_count_y; i++) {
        int y = ORIGIN_Y - i*(int)SCALE;
        if (y >= 0 && y <= WINDOW_HEIGHT) SDL_RenderDrawLine(r, 0, y, WINDOW_WIDTH, y);
    }

    // Eksenler daha koyu
    SDL_SetRenderDrawColor(r, 150, 150, 150, 255);
    SDL_RenderDrawLine(r, ORIGIN_X, 0, ORIGIN_X, WINDOW_HEIGHT);
    SDL_RenderDrawLine(r, 0, ORIGIN_Y, WINDOW_WIDTH, ORIGIN_Y);

    // SAYILARI CIZ - X ekseni (yatay) - DAHA BUYUK VE KOYU
    SDL_Color label_col = {40, 40, 40, 255};  // Cok koyu gri - neredeyse siyah
    char buf[8];
    for (int i = -step_count_x; i <= step_count_x; i++) {
        if (i == 0) continue;
        int x = ORIGIN_X + i*(int)SCALE;
        if (x < 80 || x > WINDOW_WIDTH - 80) continue;
        snprintf(buf, sizeof(buf), "%d", i);
        SDL_Surface *surf = TTF_RenderText_Blended(font, buf, label_col);
        if (surf) {
            SDL_Texture *tex = SDL_CreateTextureFromSurface(r, surf);
            SDL_Rect dst = {x - surf->w/2, ORIGIN_Y + 15, surf->w, surf->h};
            SDL_RenderCopy(r, tex, NULL, &dst);
            SDL_DestroyTexture(tex);
            SDL_FreeSurface(surf);
        }
    }

    // SAYILARI CIZ - Y ekseni (dikey) - DAHA BUYUK VE KOYU
    for (int i = -step_count_y; i <= step_count_y; i++) {
        if (i == 0) continue;
        int y = ORIGIN_Y - i*(int)SCALE;
        if (y < 80 || y > WINDOW_HEIGHT - 80) continue;
        snprintf(buf, sizeof(buf), "%d", i);
        SDL_Surface *surf = TTF_RenderText_Blended(font, buf, label_col);
        if (surf) {
            SDL_Texture *tex = SDL_CreateTextureFromSurface(r, surf);
            SDL_Rect dst = {ORIGIN_X + 15, y - surf->h/2, surf->w, surf->h};
            SDL_RenderCopy(r, tex, NULL, &dst);
            SDL_DestroyTexture(tex);
            SDL_FreeSurface(surf);
        }
    }
}

void draw_points(SDL_Renderer *r, Point2D *pts, int cnt) {
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    for (int i = 0; i < cnt; i++) {
        int sx, sy; world_to_screen(pts[i].x, pts[i].y, &sx, &sy);
        SDL_SetRenderDrawColor(r, 180, 210, 230, 60);
        SDL_Rect dot = {sx-2, sy-2, 4, 4};
        SDL_RenderFillRect(r, &dot);
    }
}

static void shorten_line_segment(Line *L, double trim_m) {
    double dx = L->end.x - L->start.x, dy = L->end.y - L->start.y;
    double len = sqrt(dx*dx + dy*dy);
    if (len <= 2*trim_m || len < 1e-6) return;
    double ux = dx/len, uy = dy/len;
    L->start.x += ux*trim_m; L->start.y += uy*trim_m;
    L->end.x -= ux*trim_m; L->end.y -= uy*trim_m;
}

void draw_lines(SDL_Renderer *r, Line *lines, int cnt) {
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    const SDL_Color palette[10] = {
        {80,150,80,255},{80,120,180,255},{200,150,80,255},{180,100,180,255},{100,180,180,255},
        {180,140,100,255},{150,100,150,255},{100,150,100,255},{150,150,100,255},{100,100,180,255}
    };
    double trim_m = 0.15;
    for (int i = 0; i < cnt; i++) {
        Line temp = lines[i];
        shorten_line_segment(&temp, trim_m);
        SDL_Color c = palette[i % 10];
        SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
        int x1, y1, x2, y2;
        world_to_screen(temp.start.x, temp.start.y, &x1, &y1);
        world_to_screen(temp.end.x, temp.end.y, &x2, &y2);
        for (int th = -1; th <= 1; th++) {
            SDL_RenderDrawLine(r, x1+th, y1, x2+th, y2);
            SDL_RenderDrawLine(r, x1, y1+th, x2, y2+th);
        }
    }
}

void draw_line_points(SDL_Renderer *r, Line *lines, int cnt, Point2D *all_pts, int total_pts) {
    const SDL_Color palette[10] = {
        {80,150,80,255},{80,120,180,255},{200,150,80,255},{180,100,180,255},{100,180,180,255},
        {180,140,100,255},{150,100,150,255},{100,150,100,255},{150,150,100,255},{100,100,180,255}
    };
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    for (int li = 0; li < cnt; li++) {
        if (!lines[li].model.inlier_indices) continue;
        SDL_Color c = palette[li % 10];
        for (int k = 0; k < lines[li].model.inlier_count; k++) {
            int orig_idx = lines[li].model.inlier_indices[k];
            for (int pi = 0; pi < total_pts; pi++) {
                if (all_pts[pi].idx == orig_idx) {
                    int sx, sy;
                    world_to_screen(all_pts[pi].x, all_pts[pi].y, &sx, &sy);
                    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, 255);
                    SDL_Rect dot = {sx-3, sy-3, 6, 6};
                    SDL_RenderFillRect(r, &dot);
                    break;
                }
            }
        }
    }
}

void draw_dashed_line(SDL_Renderer *r, int x1, int y1, int x2, int y2) {
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(r, 255, 50, 50, 220);
    double dx = x2 - x1, dy = y2 - y1;
    double distance = sqrt(dx*dx + dy*dy);
    if (distance < 1.0) return;
    int dash_len = 10, gap_len = 6;
    double step = dash_len + gap_len;
    int steps = (int)(distance / step) + 1;
    double ux = dx / distance, uy = dy / distance;
    for (int i = 0; i < steps; i++) {
        double sx = x1 + (i * step) * ux;
        double sy = y1 + (i * step) * uy;
        double ex = sx + dash_len * ux;
        double ey = sy + dash_len * uy;
        for (int th = 0; th < 2; th++) {
            SDL_RenderDrawLine(r, (int)sx+th, (int)sy, (int)ex+th, (int)ey);
        }
    }
}

void draw_robot_to_corners(SDL_Renderer *r, ValidIntersection *corners, int cnt) {
    if (cnt <= 0) return;
    int robot_x = ORIGIN_X, robot_y = ORIGIN_Y;
    for (int i = 0; i < cnt; i++) {
        int cx, cy;
        world_to_screen(corners[i].intersection_point.x, corners[i].intersection_point.y, &cx, &cy);
        draw_dashed_line(r, robot_x, robot_y, cx, cy);
    }
}

void draw_corners_with_labels(SDL_Renderer *r, ValidIntersection *corners, int cnt, TTF_Font *font) {
    if (cnt <= 0) return;
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    for (int i = 0; i < cnt; i++) {
        int sx, sy;
        world_to_screen(corners[i].intersection_point.x, corners[i].intersection_point.y, &sx, &sy);

        // Kose isaretcisi - X seklinde sari
        SDL_SetRenderDrawColor(r, 255, 220, 0, 255);
        for (int off = -8; off <= 8; off++) {
            SDL_RenderDrawPoint(r, sx+off, sy+off);
            SDL_RenderDrawPoint(r, sx+off, sy-off);
            SDL_RenderDrawPoint(r, sx+off+1, sy+off);
            SDL_RenderDrawPoint(r, sx+off+1, sy-off);
        }

        // Bilgi kutusu
        char info1[32], info2[32];
        snprintf(info1, sizeof(info1), "d%d & d%d", corners[i].lineA, corners[i].lineB);
        snprintf(info2, sizeof(info2), "%.2fm  %.0f�", corners[i].distance_from_robot, corners[i].angle_deg);

        SDL_Color white = {255, 255, 255, 255};
        SDL_Surface *surf1 = TTF_RenderText_Blended(font, info1, white);
        SDL_Surface *surf2 = TTF_RenderText_Blended(font, info2, white);

        if (surf1 && surf2) {
            int box_w = (surf1->w > surf2->w ? surf1->w : surf2->w) + 8;
            int box_h = surf1->h + surf2->h + 6;

            SDL_SetRenderDrawColor(r, 0, 0, 0, 200);
            SDL_Rect bg = {sx + 12, sy - box_h/2, box_w, box_h};
            SDL_RenderFillRect(r, &bg);

            SDL_Texture *tex1 = SDL_CreateTextureFromSurface(r, surf1);
            SDL_Texture *tex2 = SDL_CreateTextureFromSurface(r, surf2);

            SDL_Rect dst1 = {sx + 16, sy - box_h/2 + 2, surf1->w, surf1->h};
            SDL_Rect dst2 = {sx + 16, sy - box_h/2 + surf1->h + 4, surf2->w, surf2->h};

            SDL_RenderCopy(r, tex1, NULL, &dst1);
            SDL_RenderCopy(r, tex2, NULL, &dst2);

            SDL_DestroyTexture(tex1);
            SDL_DestroyTexture(tex2);
        }

        if (surf1) SDL_FreeSurface(surf1);
        if (surf2) SDL_FreeSurface(surf2);
    }
}

void draw_distance_on_line(SDL_Renderer *r, int x1, int y1, int x2, int y2, float dist, TTF_Font *font) {
    int mx = (x1 + x2) / 2;
    int my = (y1 + y2) / 2;

    char text[32];
    snprintf(text, sizeof(text), "%.2fm", dist);

    SDL_Color bg_col = {255, 200, 50,230};
    SDL_Color text_col = {0, 0, 0, 255};

    SDL_Surface *surf = TTF_RenderText_Blended(font, text, text_col);
    if (surf) {
        int box_w = surf->w + 10;
        int box_h = surf->h + 6;

        SDL_SetRenderDrawColor(r, bg_col.r, bg_col.g, bg_col.b, bg_col.a);
        SDL_Rect bg = {mx - box_w/2, my - box_h/2 - 10, box_w, box_h};
        SDL_RenderFillRect(r, &bg);

        SDL_Texture *tex = SDL_CreateTextureFromSurface(r, surf);
        SDL_Rect dst = {mx - surf->w/2, my - surf->h/2 - 10, surf->w, surf->h};
        SDL_RenderCopy(r, tex, NULL, &dst);

        SDL_DestroyTexture(tex);
        SDL_FreeSurface(surf);
    }
}

void draw_robot(SDL_Renderer *r) {
    SDL_SetRenderDrawColor(r, 255, 0, 0, 255);
    for (int rad = 6; rad <= 9; rad++) {
        for (int a = 0; a < 360; a += 10) {
            int px = ORIGIN_X + (int)(rad * cos(a * PI / 180.0));
            int py = ORIGIN_Y + (int)(rad * sin(a * PI / 180.0));
            SDL_RenderDrawPoint(r, px, py);
        }
    }
}

void draw_legend(SDL_Renderer *r, Line *lines, int line_cnt, TTF_Font *font_small) {
    int start_x = 20, start_y = 20;
    int box_width = 200;
    int row_height = 18;
    int total_rows = 1 + line_cnt*2 + 2;
    int box_height = total_rows * row_height + 10;

    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(r, 255, 255, 255, 230);
    SDL_Rect bg = {start_x, start_y, box_width, box_height};
    SDL_RenderFillRect(r, &bg);

    SDL_SetRenderDrawColor(r, 150, 150, 150, 255);
    SDL_RenderDrawRect(r, &bg);

    int y_offset = start_y + 8;

    SDL_Color black = {0, 0, 0, 255};
    SDL_Surface *title_surf = TTF_RenderText_Blended(font_small, "Gecerli Noktalar", black);
    if (title_surf) {
        SDL_Texture *title_tex = SDL_CreateTextureFromSurface(r, title_surf);
        SDL_Rect title_dst = {start_x + 8, y_offset, title_surf->w, title_surf->h};
        SDL_RenderCopy(r, title_tex, NULL, &title_dst);
        SDL_DestroyTexture(title_tex);
        SDL_FreeSurface(title_surf);
    }
    y_offset += row_height;

    SDL_SetRenderDrawColor(r, 255, 0, 0, 255);
    for (int rad = 3; rad <= 5; rad++) {
        for (int a = 0; a < 360; a += 30) {
            int px = start_x + 15 + (int)(rad * cos(a * PI / 180.0));
            int py = y_offset + 6 + (int)(rad * sin(a * PI / 180.0));
            SDL_RenderDrawPoint(r, px, py);
        }
    }

    SDL_Surface *robot_surf = TTF_RenderText_Blended(font_small, "Robot", black);
    if (robot_surf) {
        SDL_Texture *robot_tex = SDL_CreateTextureFromSurface(r, robot_surf);
        SDL_Rect robot_dst = {start_x + 35, y_offset + 1, robot_surf->w, robot_surf->h};
        SDL_RenderCopy(r, robot_tex, NULL, &robot_dst);
        SDL_DestroyTexture(robot_tex);
        SDL_FreeSurface(robot_surf);
    }
    y_offset += row_height;

    const SDL_Color palette[10] = {
        {80,150,80,255},{80,120,180,255},{200,150,80,255},{180,100,180,255},{100,180,180,255},
        {180,140,100,255},{150,100,150,255},{100,150,100,255},{150,150,100,255},{100,100,180,255}
    };

    for (int i = 0; i < line_cnt; i++) {
        SDL_Color c = palette[i % 10];

        SDL_SetRenderDrawColor(r, c.r, c.g, c.b, 255);
        for (int px = start_x + 13; px <= start_x + 18; px++) {
            for (int py = y_offset + 4; py <= y_offset + 9; py++) {
                SDL_RenderDrawPoint(r, px, py);
            }
        }

        char point_text[48];
        snprintf(point_text, sizeof(point_text), "d%d noktalari (%d)", i+1, lines[i].model.inlier_count);
        SDL_Surface *pt_surf = TTF_RenderText_Blended(font_small, point_text, black);
        if (pt_surf) {
            SDL_Texture *pt_tex = SDL_CreateTextureFromSurface(r, pt_surf);
            SDL_Rect pt_dst = {start_x + 35, y_offset + 1, pt_surf->w, pt_surf->h};
            SDL_RenderCopy(r, pt_tex, NULL, &pt_dst);
            SDL_DestroyTexture(pt_tex);
            SDL_FreeSurface(pt_surf);
        }
        y_offset += row_height;

        SDL_SetRenderDrawColor(r, c.r, c.g, c.b, 255);
        for (int th = 0; th < 2; th++) {
            SDL_RenderDrawLine(r, start_x + 12, y_offset + 6 + th, start_x + 25, y_offset + 6 + th);
        }

        double dx = lines[i].end.x - lines[i].start.x;
        double dy = lines[i].end.y - lines[i].start.y;
        double len = sqrt(dx*dx + dy*dy);

        char line_text[32];
        snprintf(line_text, sizeof(line_text), "d%d (%.2fm)", i+1, len);
        SDL_Surface *ln_surf = TTF_RenderText_Blended(font_small, line_text, black);
        if (ln_surf) {
            SDL_Texture *ln_tex = SDL_CreateTextureFromSurface(r, ln_surf);
            SDL_Rect ln_dst = {start_x + 35, y_offset + 1, ln_surf->w, ln_surf->h};
            SDL_RenderCopy(r, ln_tex, NULL, &ln_dst);
            SDL_DestroyTexture(ln_tex);
            SDL_FreeSurface(ln_surf);
        }
        y_offset += row_height;
    }

    SDL_SetRenderDrawColor(r, 255, 220, 0, 255);
    for (int off = -5; off <= 5; off++) {
        SDL_RenderDrawPoint(r, start_x + 15 + off, y_offset + 6 + off);
        SDL_RenderDrawPoint(r, start_x + 15 + off, y_offset + 6 - off);
    }

    SDL_Surface *corner_surf = TTF_RenderText_Blended(font_small, "60+ Kesisim", black);
    if (corner_surf) {
        SDL_Texture *corner_tex = SDL_CreateTextureFromSurface(r, corner_surf);
        SDL_Rect corner_dst = {start_x + 35, y_offset + 1, corner_surf->w, corner_surf->h};
        SDL_RenderCopy(r, corner_tex, NULL, &corner_dst);
        SDL_DestroyTexture(corner_tex);
        SDL_FreeSurface(corner_surf);
    }
    y_offset += row_height;

    SDL_SetRenderDrawColor(r, 255, 50, 50, 220);
    int dash_x = start_x + 12;
    for (int i = 0; i < 3; i++) {
        SDL_RenderDrawLine(r, dash_x + i*4, y_offset + 6, dash_x + i*4 + 2, y_offset + 6);
        SDL_RenderDrawLine(r, dash_x + i*4, y_offset + 7, dash_x + i*4 + 2, y_offset + 7);
    }

    SDL_Surface *dist_surf = TTF_RenderText_Blended(font_small, "Mesafe Cizgisi", black);
    if (dist_surf) {
        SDL_Texture *dist_tex = SDL_CreateTextureFromSurface(r, dist_surf);
        SDL_Rect dist_dst = {start_x + 35, y_offset + 1, dist_surf->w, dist_surf->h};
        SDL_RenderCopy(r, dist_tex, NULL, &dist_dst);
        SDL_DestroyTexture(dist_tex);
        SDL_FreeSurface(dist_surf);
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2) { printf("Kullanim: %s <toml_file>\n", argv[0]); return 1; }
    srand((unsigned int)time(NULL));

    printf("        LIDAR KOSE TESPITI - GORSEL ANALIZ           \n");

    LidarData lidar;
    if (!load_lidar_data(argv[1], &lidar)) return 1;

    Point2D *points = malloc(sizeof(Point2D)*MAX_POINTS);
    if (!points) { printf("Bellek hatasi!\n"); return 1; }
    int valid_count = filter_and_convert_ranges(&lidar, points);
    int original_count = valid_count;

    Line *lines = malloc(sizeof(Line)*MAX_LINES);
    if (!lines) { free(points); printf("Bellek hatasi!\n"); return 1; }
    int remaining_pts = valid_count;
    int line_count = extract_all_lines(points, &remaining_pts, lines, MAX_LINES);

    ValidIntersection *corners = malloc(sizeof(ValidIntersection)*100);
    if (!corners) { free(points); free(lines); printf("Bellek hatasi!\n"); return 1; }
    int corner_count = 0;
    for (int i = 0; i < line_count; i++) {
        for (int j = i+1; j < line_count; j++) {
            Point2D inter;
            if (isLineIntersecting(lines[i], lines[j], &inter)) {
                double angleDeg = calculateAngleBetweenLines(lines[i], lines[j])*(180.0/PI);
                if (angleDeg >= MIN_CORNER_ANGLE) {
                    corners[corner_count].intersection_point = inter;
                    corners[corner_count].angle_deg = (float)angleDeg;
                    corners[corner_count].distance_from_robot = (float)sqrt(inter.x*inter.x + inter.y*inter.y);
                    corners[corner_count].lineA = i+1;
                    corners[corner_count++].lineB = j+1;
                }
            }
        }
    }

    printf("SONUCLAR:\n");
    printf("Toplam Nokta Sayisi    : %d\n", original_count);
    printf("Tespit Edilen Dogru    : %d\n", line_count);
    printf("Bulunan Kose Noktalari : %d (>= %d derece)\n\n", corner_count, (int)MIN_CORNER_ANGLE);

    printf("KOSE DETAYLARI:\n");
    for (int i = 0; i < corner_count; i++) {
        printf("Kose #%d:\n", i+1);
        printf("  Dogrular: D%d & D%d\n", corners[i].lineA, corners[i].lineB);
        printf("  Aci      : %.1f derece\n", corners[i].angle_deg);
        printf("  Mesafe   : %.2f m (robottan)\n", corners[i].distance_from_robot);
        printf("  Konum    : (%.2f, %.2f)\n\n",
               corners[i].intersection_point.x,
               corners[i].intersection_point.y);
    }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        printf("SDL baslatma hatasi: %s\n", SDL_GetError());
        for (int i = 0; i < line_count; i++) if (lines[i].model.inlier_indices) free(lines[i].model.inlier_indices);
        free(corners); free(lines); free(points);
        return 1;
    }

    if (TTF_Init() != 0) {
        printf("TTF baslatma hatasi: %s\n", TTF_GetError());
        SDL_Quit();
        for (int i = 0; i < line_count; i++) if (lines[i].model.inlier_indices) free(lines[i].model.inlier_indices);
        free(corners); free(lines); free(points);
        return 1;
    }

// IKI FARKLI FONT BOYUTU YUKLEYELIM
TTF_Font *font_normal = NULL;  // Eksen sayilari icin (16pt)
TTF_Font *font_small = NULL;   // Legend icin (11pt)

const char *font_paths[] = {
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    "/usr/share/fonts/TTF/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    "/System/Library/Fonts/Helvetica.ttc",
    "C:\\Windows\\Fonts\\arial.ttf",
    NULL
};

for (int i = 0; font_paths[i] != NULL && font_normal == NULL; i++) {
    font_normal = TTF_OpenFont(font_paths[i], 14);  // Eksen icin
    if (font_normal) {
        font_small = TTF_OpenFont(font_paths[i], 11);  // Legend icin
    }
}

if (!font_normal || !font_small) {
    printf("HATA: Hicbir font bulunamadi!\n");
    printf("Lutfen sisteminize font yukleyin:\n");
    printf("  Ubuntu/Debian: sudo apt-get install fonts-dejavu\n");
    printf("  Fedora: sudo dnf install dejavu-sans-fonts\n");
    TTF_Quit();
    SDL_Quit();
    for (int i = 0; i < line_count; i++) if (lines[i].model.inlier_indices) free(lines[i].model.inlier_indices);
    free(corners); free(lines); free(points);
    return 1;
}

    SDL_Window *win = SDL_CreateWindow("LiDAR Gorsellestirme - Robot Ortada",
                                       SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                       WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    if (!win) {
        printf("Pencere olusturma hatasi: %s\n", SDL_GetError());
        TTF_CloseFont(font_normal);
        TTF_CloseFont(font_small);
        TTF_Quit();
        SDL_Quit();
        for (int i = 0; i < line_count; i++) if (lines[i].model.inlier_indices) free(lines[i].model.inlier_indices);
        free(corners); free(lines); free(points);
        return 1;
    }

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (!ren) {
        printf("Renderer olusturma hatasi: %s\n", SDL_GetError());
        SDL_DestroyWindow(win);
        TTF_CloseFont(font_normal);
        TTF_CloseFont(font_small);
        TTF_Quit();
        SDL_Quit();
        for (int i = 0; i < line_count; i++) if (lines[i].model.inlier_indices) free(lines[i].model.inlier_indices);
        free(corners); free(lines); free(points);
        return 1;
    }

    int running = 1;
    SDL_Event e;

    ValidIntersection selected_corners[1];
    int selected_count = 0;
    if (corner_count > 0) {
        int min_idx = 0;
        float min_dist = corners[0].distance_from_robot;
        for (int ci = 1; ci < corner_count; ci++) {
            if (corners[ci].distance_from_robot < min_dist) {
                min_dist = corners[ci].distance_from_robot;
                min_idx = ci;
            }
        }
        selected_corners[0] = corners[min_idx];
        selected_count = 1;
    }

    while (running) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) running = 0;
        }

        draw_grid(ren, font_normal);
        draw_points(ren, points, original_count);
        draw_lines(ren, lines, line_count);
        draw_line_points(ren, lines, line_count, points, original_count);
        draw_robot_to_corners(ren, selected_corners, selected_count);

        if (selected_count > 0) {
            int rx = ORIGIN_X, ry = ORIGIN_Y;
            int cx, cy;
            world_to_screen(selected_corners[0].intersection_point.x,
                          selected_corners[0].intersection_point.y, &cx, &cy);
            draw_distance_on_line(ren, rx, ry, cx, cy, selected_corners[0].distance_from_robot, font_small);
        }

        draw_corners_with_labels(ren, selected_corners, selected_count, font_small);
        draw_robot(ren);
        draw_legend(ren, lines, line_count, font_small);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    TTF_CloseFont(font_normal);
    TTF_CloseFont(font_small);
    TTF_Quit();
    SDL_Quit();

    for (int i = 0; i < line_count; i++) if (lines[i].model.inlier_indices) free(lines[i].model.inlier_indices);
    free(lines); free(points); free(corners);
    return 0;
}

